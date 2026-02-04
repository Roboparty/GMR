import argparse
import json
import pathlib
import os
import multiprocessing as mp

import numpy as np
from scipy.spatial.transform import Rotation as R
from natsort import natsorted
from rich import print
import torch

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.smpl import load_smplx_file, get_smplx_data_offline_fast
from general_motion_retargeting.kinematics_model import KinematicsModel
from general_motion_retargeting import IK_CONFIG_ROOT
import gc
import time
import psutil
import tracemalloc
import joblib

def check_memory(threshold_gb=1):  # adjust based on your available memory
    mem = psutil.virtual_memory()
    used_memory_gb = (mem.total - mem.available) / (1024 ** 3)
    available_memory_gb = mem.available / (1024 ** 3)
    if available_memory_gb < threshold_gb:
        print(f"[WARNING] Memory usage:{used_memory_gb:.2f} GB, available:{available_memory_gb:.2f} GB, exceeding the threshold of {threshold_gb} GB.")
        return True
    return False


HERE = pathlib.Path(__file__).parent


def process_file(smplx_file_path, tgt_file_path, tgt_robot, SMPLX_FOLDER, tgt_folder, save_as_pkl, save_as_npz, save_as_txt, save_as_csv, total_files, verbose=False):
    def log_memory(message):
        if verbose:
            process = psutil.Process(os.getpid())
            memory_usage = process.memory_info().rss / (1024 ** 3)  # Convert to GB
            print(f"[MEMORY] {message}: {memory_usage:.2f} GB")
    
    # Start memory tracking if verbose
    if verbose:
        tracemalloc.start()
        
    # Initial checks (with optional logging)
    log_memory("Initial memory usage")
    
    # Set save flags
    PKL = save_as_pkl
    NPZ = save_as_npz
    TXT = save_as_txt
    CSV = save_as_csv
    
    num_pause = 0
    while check_memory():
        print(f"[PAUSE] Paused processing {smplx_file_path} to prevent memory overflow. num_pause: {num_pause}")
        time.sleep(20)
        num_pause += 1
        if num_pause > 5:
            print(f"[ERROR] Memory usage is still high after 10 pauses. Exiting.")
            return

    try:
        smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(smplx_file_path, SMPLX_FOLDER)
        mocap_frame_rate = smplx_data["mocap_frame_rate"]
        log_memory("After loading SMPL-X data")
    except Exception as e:
        print(f"Error loading {smplx_file_path}: {e}")
        return
    
    src_fps = smplx_data["mocap_frame_rate"].item()
    tgt_fps = src_fps
    try:
        smplx_frame_data_list, aligned_fps = get_smplx_data_offline_fast(smplx_data, body_model, smplx_output, tgt_fps=tgt_fps)
    except Exception as e:
        print(f"Error processing {smplx_file_path}: {e}")
        return
    
    # retarget
    retargeter = GMR(
        src_human="smplx",
        tgt_robot=tgt_robot,
        actual_human_height=actual_human_height,
    )
    qpos_list = []
    for smplx_frame_data in smplx_frame_data_list:
        qpos = retargeter.retarget(smplx_frame_data)
        qpos_list.append(qpos.copy())

    qpos_list = np.array(qpos_list)
    log_memory("After retargeting")
    
    device = "cuda:0" # or cpu
    
    kinematics_model = KinematicsModel(retargeter.xml_file, device=device)

    root_pos = qpos_list[:, :3]
    root_rot = qpos_list[:, 3:7]
    root_rot[:, [0, 1, 2, 3]] = root_rot[:, [1, 2, 3, 0]] # xyzw to wxyz
    dof_pos = qpos_list[:, 7:]
    num_frames = root_pos.shape[0]

    body_names = kinematics_model.body_names
    dof_names = kinematics_model.dof_names
    
    HEIGHT_ADJUST = True
    if HEIGHT_ADJUST:
        # height adjust to ensure the lowerset part is on the ground
        body_pos, body_rot = kinematics_model.forward_kinematics(torch.from_numpy(root_pos).to(device=device, dtype=torch.float), 
                                                        torch.from_numpy(root_rot).to(device=device, dtype=torch.float), 
                                                        torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)) # TxNx3
        # ground_offset = -0.05
        lowerst_height = torch.min(body_pos[..., 2]).item()
        root_pos[:, 2] = root_pos[:, 2] - lowerst_height # make sure motion on the ground
        
    ROOT_ORIGIN_OFFSET = True
    if ROOT_ORIGIN_OFFSET:
        # offset using the first frame
        root_pos[:, :2] -= root_pos[0, :2]

    fk_root_pos = torch.zeros((num_frames, 3), device=device)
    fk_root_rot = torch.zeros((num_frames, 4), device=device)
    fk_root_rot[:, -1] = 1.0

    local_body_pos, local_body_rot = kinematics_model.forward_kinematics(
        fk_root_pos, fk_root_rot, torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
    )

    log_memory("After forward kinematics")
    
    motion_data = {
        "fps": aligned_fps,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_names": dof_names,
        "body_names": body_names,
        # "link_body_list": body_names, # to be modified if needed in AMP(legged_lab)
        "dof_positions": dof_pos,
        "dof_pos": dof_pos,
        "body_positions": body_pos,
        "body_rotations": body_rot,
        "local_body_pos": body_pos,
    }


    # helpers for saving in different formats
    def to_numpy_compatible(d):
        """Return a dict with numpy arrays / python scalars suitable for np.savez."""
        out = {}
        for k, v in d.items():
            # torch tensor -> numpy
            if isinstance(v, torch.Tensor):
                out[k] = v.detach().cpu().numpy()
            # numpy array -> keep
            elif isinstance(v, np.ndarray):
                out[k] = v
            # list/tuple -> convert to numpy
            elif isinstance(v, (list, tuple)):
                try:
                    out[k] = np.array(v)
                except Exception:
                    out[k] = np.array([str(v)])
            # basic types (int/float/str/... including numpy scalar)
            elif isinstance(v, (np.generic,)):
                try:
                    out[k] = np.array(v).tolist()
                except Exception:
                    out[k] = v
            else:
                out[k] = v
        return out

    os.makedirs(os.path.dirname(tgt_file_path), exist_ok=True)

    base_no_ext = os.path.splitext(tgt_file_path)[0]
    npz_path = base_no_ext + ".npz"
    pkl_path = base_no_ext + ".pkl"
    txt_path = base_no_ext + ".txt"
    os.makedirs(os.path.dirname(npz_path), exist_ok=True)

    # numpy-compatible dict
    try:
        npz_dict = to_numpy_compatible(motion_data)
    except Exception as e:
        print(f"[ERROR] Converting to numpy-compatible failed for {npz_path}: {e}")
        npz_dict = {}
    
    if NPZ:
        # 1) Save npz
        try:
            np.savez_compressed(npz_path, **npz_dict)
            print(f"Saved to {npz_path}")
        except Exception as e:
            print(f"[ERROR] Saving .npz failed for {npz_path}: {e}")
            
    if PKL:
        # 2) Save pkl
        try:
            joblib.dump(npz_dict, pkl_path)
            print(f"Saved to {pkl_path}")
        except Exception as _e:
            print(f"[WARN] joblib dump failed for {pkl_path}: {_e}")

    if TXT:
        # 3) Save txt
        try:
            with open(txt_path, "w") as f:
                for k, v in npz_dict.items():
                    f.write(f"{k}: {v}\n")
            print(f"Saved to {txt_path}")
        except Exception as e:
            print(f"[ERROR] Saving .txt failed for {txt_path}: {e}")
            
    if CSV:
        # 4) Save csv
        try:
            def export_to_csv(root_pos, root_rot, dof_pos, filename):
                num_frames = root_pos.shape[0]
                with open(filename, 'w') as f:
                    for i in range(num_frames):
                        row = [f"{root_pos[i, j]:.6f}" for j in range(3)]
                        row += [f"{root_rot[i, j]:.6f}" for j in range(4)]
                        row += [f"{dof_pos[i, j]:.6f}" for j in range(dof_pos.shape[1])]
                        f.write(','.join(row) + '\n')
            csv_path = base_no_ext + ".csv"
            export_to_csv(root_pos, root_rot, dof_pos, csv_path) # 8 14
            print(f"Saved to {csv_path}")
            
        except Exception as e:
            print(f"[ERROR] Saving .csv failed for {csv_path}: {e}")
                
    if verbose:
        # Get memory snapshot
        snapshot = tracemalloc.take_snapshot()
        top_stats = snapshot.statistics('lineno')
        
        print("\nTop 10 memory-consuming lines:")
        for stat in top_stats[:10]:
            print(stat)
        
        tracemalloc.stop()
        
    # clean cache
    torch.cuda.empty_cache()
    gc.collect()
    


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2",
                 "booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1", 
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1",
                "pnd_adam_lite", "openlong", "roboparty_atom01", "roboparty_atom02"],
        default="roboparty_atom01",
        # default="roboparty_atom02",
        # default="unitree_g1",
    )
    parser.add_argument("--src_folder", type=str,
                        default="/home/msi/Desktop/src",
                        )
    parser.add_argument("--tgt_folder", type=str,
                        default="/home/msi/Desktop/tgt",
                        )
    parser.add_argument(
        "--save_as_pkl",
        default=True, # True or False
        help="whether to save the robot motion as pkl format.",
    )

    parser.add_argument(
        "--save_as_txt",
        default=True, # True or False
        help="whether to save the robot motion as txt format.",
    )
    
    parser.add_argument(
        "--save_as_csv", 
        default=True, # True or False
        help="whether to save the robot motion as csv format.",
    )
    
    parser.add_argument(
        "--save_as_npz",
        default=True, # True or False
        help="whether to save the robot motion as npz format.",
    )
    parser.add_argument("--override", default=True, action="store_true")
    parser.add_argument("--num_cpus", default=5, type=int)
    args = parser.parse_args()
    
    # print the total number of cpus and gpus
    print(f"Total CPUs: {mp.cpu_count()}")
    print(f"Using {args.num_cpus} CPUs.")
    
    src_folder = args.src_folder
    tgt_folder = args.tgt_folder

    SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"
    hard_motions_folder = HERE / ".." / "assets" / "hard_motions"

    verbose = False

    hard_motions_paths = [hard_motions_folder / "0.txt", 
                          hard_motions_folder / "1.txt"]
    hard_motions = []
    for hard_motions_path in hard_motions_paths:
        with open(hard_motions_path, "r") as f:
            for line in f:
                if "Motion:" in line:
                    motion_path = line.split(":")[1].strip()
                else:
                    continue
                motion_path = motion_path.split(",")[0].strip().split(".")[0]
                hard_motions.append(motion_path)
                
                
    args_list = []
    for dirpath, _, filenames in os.walk(src_folder):
        for filename in natsorted(filenames):
            if filename.endswith("_stagei.npz"):
                continue
            if filename.endswith((".pkl", ".npz")):
                smplx_file_path = os.path.join(dirpath, filename)

                rel_path = os.path.relpath(smplx_file_path, src_folder)
                tgt_file_path = os.path.join(tgt_folder, rel_path)

                if not os.path.exists(tgt_file_path) or args.override:
                    args_list.append((smplx_file_path, tgt_file_path, args.robot, SMPLX_FOLDER, tgt_folder, args.save_as_pkl, args.save_as_npz, args.save_as_txt, args.save_as_csv))
    print("full args_list:", len(args_list))
    
    # remove hard and infeasible motions
    exclude_file_content = ["BMLrub", "EKUT", "crawl", "_lie", "upstairs", "downstairs"]
    
    new_args_list = []
    for arguments in args_list:
        motion_name = arguments[0].split("/")[-1].split('.')[0]
        if motion_name in hard_motions:
            continue
        if any(content in motion_name for content in exclude_file_content):
            continue
        new_args_list.append(arguments)
    args_list = new_args_list
    
    
    print("new args_list:", len(args_list))
    
    total_files = len(args_list)
    print(f"Total number of files to process: {total_files}")
    with mp.Pool(args.num_cpus) as pool:
        pool.starmap(process_file, [args + (total_files, verbose) for args in args_list])

    print("Done. Saved to ", tgt_folder)


if __name__ == "__main__":
    main()
