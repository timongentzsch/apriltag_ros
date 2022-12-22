import numpy as np
import pandas as pd
import yaml
import argparse
from scipy.optimize import minimize

# HELPER FUNCTIONS
# ------------------------------------------------------------------------------

# get rotation matrix from quaternion
def get_rotation_matrix(q):
    q = q / np.linalg.norm(q)
    q_x = q[0]
    q_y = q[1]
    q_z = q[2]
    q_w = q[3]
    R = np.array([[1 - 2 * q_y ** 2 - 2 * q_z ** 2, 2 * q_x * q_y - 2 * q_z * q_w, 2 * q_x * q_z + 2 * q_y * q_w],
                    [2 * q_x * q_y + 2 * q_z * q_w, 1 - 2 * q_x ** 2 - 2 * q_z ** 2, 2 * q_y * q_z - 2 * q_x * q_w],
                    [2 * q_x * q_z - 2 * q_y * q_w, 2 * q_y * q_z + 2 * q_x * q_w, 1 - 2 * q_x ** 2 - 2 * q_y ** 2]])
    return R


# get quaternion from rotation matrix
def get_quaternion(R):
    q = np.zeros(4)
    q[0] = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    q[1] = (R[2, 1] - R[1, 2]) / (4 * q[0])
    q[2] = (R[0, 2] - R[2, 0]) / (4 * q[0])
    q[3] = (R[1, 0] - R[0, 1]) / (4 * q[0])
    return q

# get rigid transformation matrix from rotation matrix and translation vector
def get_rigid_transformation_matrix(R, t):
    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

# get inverse rigid transformation matrix
def get_inverse_rigid_transformation_matrix(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.identity(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

# compute average quaternion from a list of quaternions with eigenvector method
def quatWAvgMarkley(Q, weights):
    '''
    Averaging Quaternions.

    Arguments:
        Q(ndarray): an Mx4 ndarray of quaternions.
        weights(list): an M elements list, a weight for each quaternion.
    '''

    # Form the symmetric accumulator matrix
    A = np.zeros((4, 4))
    M = Q.shape[0]
    wSum = 0

    for i in range(M):
        q = Q[i, :]
        w_i = weights[i]
        A += w_i * (np.outer(q, q)) # rank 1 update
        wSum += w_i

    # scale
    A /= wSum

    # Get the eigenvector corresponding to largest eigen value
    return np.linalg.eigh(A)[1][:, -1]

# DETECTION CLASSES
# ------------------------------------------------------------------------------
class detection:
    def __init__(self, t, q):
        self.t = t
        self.q = q
        self.R = get_rotation_matrix(q)
        self.R_inv = np.linalg.inv(self.R)
        self.T = get_rigid_transformation_matrix(self.R, self.t)
        self.T_inv = get_inverse_rigid_transformation_matrix(self.T)
    # show tag pose when printing
    def __str__(self):
        return f't = {self.t}, q = {self.q}'

class rel_detection:
    def __init__(self, T):
        self.T = T
        self.T_inv = get_inverse_rigid_transformation_matrix(self.T)
        self.R = self.T_inv[:3, :3]
        self.t = self.T_inv[:3, 3]
        self.q = get_quaternion(self.R)
    # show tag pose when printing
    def __str__(self):
        return f't = {self.t}, q = {self.q}'

# Calculate rel poses from tag poses
# ------------------------------------------------------------------------------

def bundle_calib(CSV_PATH, OUTPUT_PATH, MASTER_TAG_ID, ROUND_DIGITS):

    # VARIABLES
    # ------------------------------------------------------------------------------
    # CSV_PATH: path to CSV file containing tag poses
    # MASTER_TAG_ID: tag id of the master tag
    # OUTPUT_PATH: path to output file + filename
    # ROUND_DIGITS: number of digits to round to in yaml file

    # import csv into pandas dataframe
    df = pd.read_csv(CSV_PATH)

    # convert dataframe to numpy array structure
    images = {}

    # for each unique in Path column, get df row
    for unique in df.Path.unique():
        sub_df = df[df.Path == unique].values
        tag_detections = {}
        for i, row in enumerate(sub_df):
            tag_detections[row[1]] = detection(row[3:6], row[6:])
        images[row[0]] = tag_detections

    # remove all images that don't have the master tag
    for key in list(images.keys()):
        if MASTER_TAG_ID not in images[key]:
            del images[key]

    # get all unique tag ids
    unique_ids = df.tag_id.unique()
    # initialize empty array to store all tag poses
    master_tag_detections_per_id = {}
    for id in unique_ids:
        master_tag_detections_per_id[id] = []


    # get relative poses of all tags in master tag frame
    master_tag_detections = {}
    for image in images.keys():
        master_tag_detections[image] = {}
        for tag_id in images[image].keys():
            master_tag_detections[image][tag_id] = images[image][tag_id].T_inv @ images[image][MASTER_TAG_ID].T
            master_tag_detections_per_id[tag_id].append(rel_detection(master_tag_detections[image][tag_id]))

    # Nelder-Mead optimization for translation vector
    # ------------------------------------------------------------------------------

    final_rel_quats = {}
    t_init = {}
    for id in unique_ids:
        # compute average quaternion for each tag id
        final_rel_quats[id] = quatWAvgMarkley(np.array([i.q for i in master_tag_detections_per_id[id]]), np.ones(len(master_tag_detections_per_id[MASTER_TAG_ID])))
        # average translation vector as initial guess
        t_init[id] = np.mean([i.t for i in master_tag_detections_per_id[id]], axis=0)

    final_rel_pos = t_init
    # compute (geometric) median position of each tag in master tag frame

    def least_squares_func(x, rel_pos):
        diff = (x - rel_pos)**2
        diff_sqrt = np.sqrt(np.sum(diff, axis=1))
        return np.sum(diff_sqrt)

    for id in unique_ids:
        x0 = t_init[id]
        res = minimize(fun=least_squares_func, x0=x0, args=(np.array([rel_pos.t for rel_pos in master_tag_detections_per_id[id]])), method='Nelder-Mead')
        print(f"res.x = {res.x}, x0 = {x0}")
        if res.success:
            final_rel_pos[id] = res.x
        else:
            print(f"{id} optimization failed")

    # write to yaml file
    # ------------------------------------------------------------------------------

    # round values to X decimal places
    for id in final_rel_quats.keys():
        final_rel_quats[id] = np.round(final_rel_quats[id], ROUND_DIGITS)
    for id in final_rel_pos.keys():
        final_rel_pos[id] = np.round(final_rel_pos[id], ROUND_DIGITS)

    output_dict = {}
    output_dict['bundle'] = {}
    slavecounter = 1 # counter for slave tag ids 

    # get size of tags in df
    id_sizes = {}
    for id in unique_ids:
        sub_df = df[df.tag_id == id].values
        id_sizes[id] = sub_df[0][2]

    for id in unique_ids:
        if id == MASTER_TAG_ID:
            output_dict["bundle"]["master"] = {"id": int(id), "size": id_sizes[id], "translation": f'{final_rel_pos[id].tolist()}', "quaternion": f'{final_rel_quats[id].tolist()}'}
        else:
            output_dict["bundle"][f"slave_{slavecounter}"] = {"id": int(id), "size": id_sizes[id], "translation": f'{final_rel_pos[id].tolist()}', "quaternion": f'{final_rel_quats[id].tolist()}'}
            slavecounter += 1

    yaml_dump = yaml.dump(output_dict, default_style=False)
    # remove all ' from test string
    yaml_dump = yaml_dump.replace("'", "")

    print(f"Saving file to {OUTPUT_PATH}")
    print(yaml_dump)
    with open(OUTPUT_PATH, "w") as f:
        f.write(yaml_dump)

def main(args=None):
    parser = argparse.ArgumentParser(description='Script to compute relative poses of tags in master tag frame')
    parser.add_argument('-i', '--input', help='path to input file', required=True, type=str)
    parser.add_argument('-o', '--output', help='path to output file', required=True, type=str)
    parser.add_argument('-m', '--master', help='master tag id', required=False, type=int, default=95)
    parser.add_argument('-d', '--digits', help='number of digits to round to', required=False, type=int, default=5)
    args = parser.parse_args()
    bundle_calib(args.input, args.output, args.master, args.digits)


if __name__ == "__main__":
    main()