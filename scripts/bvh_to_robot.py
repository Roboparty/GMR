import argparse
import pathlib
import time
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.lafan1 import load_bvh_file
from general_motion_retargeting.kinematics_model import KinematicsModel
import torch
from rich import print
import os
import numpy as np
import sys
import select
import termios
import tty
import joblib
from tools import axis_angle_from_quat, quat_conjugate, quat_mul, quat_slerp


def _so3_derivative(rotations: torch.Tensor, dt: float) -> torch.Tensor:
    """Computes the derivative of a sequence of SO3 rotations.

    Args:
        rotations: shape (B, 4).
        dt: time step.
    Returns:
        shape (B, 3).
    """
    q_prev, q_next = rotations[:-2], rotations[2:]
    q_rel = quat_mul(q_next, quat_conjugate(q_prev))  # shape (B−2, 4)

    omega = axis_angle_from_quat(q_rel) / (2.0 * dt)  # shape (B−2, 3)
    omega = torch.cat([omega[:1], omega, omega[-1:]], dim=0)  # repeat first and last sample
    return omega


if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bvh_file",
        help="BVH motion file to load.",
        # required=True,
        # default="/home/msi/Desktop/lafan1/walk1_subject2.bvh",
        default="/home/msi/Downloads/demo_dataset/20251216_seat-2_012_001.bvh",
        type=str,
    )

    parser.add_argument(
        "--save_slice",
        default=False, # True or False
        help="Whether to save a slice of the robot motion.",
    )

    parser.add_argument(
        "--slice_motion_start_end",
        default=[700, 850],
        help="Whether to save a slice of the robot motion.",
    )
    
    parser.add_argument(
        "--save_as_pkl",
        default=False, # True or False
        help="whether to save the robot motion as pkl format.",
    )

    parser.add_argument(
        "--save_as_txt",
        default=False, # True or False
        help="whether to save the robot motion as txt format.",
    )
    
    parser.add_argument(
        "--save_as_csv", 
        default=False, # True or False
        help="whether to save the robot motion as csv format.",
    )
    
    parser.add_argument(
        "--save_as_npz",
        default=False, # True or False
        help="whether to save the robot motion as npz format.",
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "fourier_n1", "engineai_pm01","roboparty_atom01"],
        # default="roboparty_atom01",
        # default="roboparty_atom02",
        default="unitree_g1",
        # default="engineai_pm01",
    )
        
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )

    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
    )
    
    args_cli = parser.parse_args()
    parser.add_argument(
        "--save_path",
        default=f"{args_cli.robot}_gmr",
        help="Path to save the robot motion.",
    )
    
    args = parser.parse_args()


    if args.save_path is not None:
        os.makedirs(args.save_path, exist_ok=True)

    
    # Load SMPLX trajectory
    lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file)
    src_fps = 30  # LAFAN1 data is typically 30 FPS
    
    # Initialize the retargeting system
    retargeter = GMR(
        src_human="bvh_nokov", # bvh_lafan1
        tgt_robot=args.robot,
        actual_human_height=actual_human_height,
    )

    motion_fps = 30
    
    robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                            motion_fps=motion_fps,
                                            transparent_robot=0,
                                            record_video=args.record_video,
                                            video_path=args.video_path,
                                            # video_width=2080,
                                            # video_height=1170
                                            )
    
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        
        
    qpos_list = []
        
    # Create tqdm progress bar for the total number of frames
    
    # Start the viewer

    START_FRAME = args.slice_motion_start_end[0] # e.g., 180
    END_FRAME = args.slice_motion_start_end[1]

    SAVE_SLICE = args.save_slice
    if SAVE_SLICE == True:
        i = START_FRAME
        n_frames_total = END_FRAME
    else:
        i = 0
        n_frames_total = len(lafan1_data_frames)
        
        
    # False 则保存全部帧
    SAVE_SLICE = args.save_slice
    if SAVE_SLICE == True:
        if START_FRAME >= len(lafan1_data_frames):
            print(f"START_FRAME {START_FRAME} exceeds total frames {len(lafan1_data_frames)}. Adjusting to {len(lafan1_data_frames)-1}.")
            START_FRAME = len(lafan1_data_frames) - 1
        if END_FRAME > len(lafan1_data_frames):
            print(f"END_FRAME {END_FRAME} exceeds total frames {len(lafan1_data_frames)}. Adjusting to {len(lafan1_data_frames)}.")
            END_FRAME = len(lafan1_data_frames)
        i = START_FRAME
        n_frames_total = END_FRAME
    else:
        i = 0
        n_frames_total = len(lafan1_data_frames)        
        
    # --- 最小改动：在终端设置 cbreak，按空格切换 paused ---
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    paused = False

    try:
        while i < n_frames_total:
            
            # 非阻塞读取按键
            dr, _, _ = select.select([sys.stdin], [], [], 0.0)
            if dr:
                ch = sys.stdin.read(1)
                if ch == " ":
                    paused = not paused
                    print(f"Paused: {paused}")
                elif ch in ("\x03", "q"):
                    print("Exit requested.")
                    break

            if paused:
                time.sleep(0.05)
                continue
            
            print(f"Processing frame {i+1}/{n_frames_total}")
            
            # FPS measurement
            fps_counter += 1
            current_time = time.time()
            if current_time - fps_start_time >= fps_display_interval:
                actual_fps = fps_counter / (current_time - fps_start_time)
                print(f"Actual rendering FPS: {actual_fps:.2f}")
                fps_counter = 0
                fps_start_time = current_time
                
            # Update task targets.
            smplx_data = lafan1_data_frames[i]

            # retarget
            qpos = retargeter.retarget(smplx_data)

            # visualize
            robot_motion_viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retargeter.scaled_human_data,
                # rate_limit=args.rate_limit, # 这里改为 False 保持高速运行
                rate_limit=False,
                human_pos_offset=np.array([0.0, 0.0, 0.0])
            )

            i += 1

            if args.save_path is not None:
                qpos_list.append(qpos.copy())
    
    finally:
        # 恢复终端设置
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
      
        
    device = "cuda:0"
    kinematics_model = KinematicsModel(retargeter.xml_file, device=device)
  
    # Ensure qpos_list is a numpy array before slicing (fix TypeError)
    if 'qpos_list' in locals() and isinstance(qpos_list, list):
        qpos_list = np.array(qpos_list)
        
    if SAVE_SLICE:
        print(f"Saved frames from {START_FRAME} to {END_FRAME}, total {END_FRAME - START_FRAME} frames.")
    print(f"Total frames to process: {len(qpos_list)}")
    
    root_pos = qpos_list[:, :3]
    root_rot = qpos_list[:, 3:7]
    # root_rot[:, [0, 1, 2, 3]] = root_rot[:, [1, 2, 3, 0]]
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
        ground_offset = 0.01
        lowerst_height_body_pos = torch.min(body_pos[..., 2]).item()    
        root_pos[:, 2] = root_pos[:, 2] - lowerst_height_body_pos

    ROOT_ORIGIN_OFFSET = True
    if ROOT_ORIGIN_OFFSET:
        # offset using the first frame
        root_pos[:, :2] -= root_pos[0, :2]

    fk_root_pos = torch.zeros((num_frames, 3), device=device)
    fk_root_rot = torch.zeros((num_frames, 4), device=device)
    fk_root_rot[:, -1] = 1.0

    local_body_pos, _ = kinematics_model.forward_kinematics(
        fk_root_pos, fk_root_rot, torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
    )
    
    print("Generating visualization with adjusted motion...")
    if SAVE_SLICE == True:
        i = 0
        n_frames_total = END_FRAME - START_FRAME + 1
    else:
        i = 0
        n_frames_total = len(lafan1_data_frames)
    
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    

    # --- 在终端设置 cbreak，按空格切换 paused ---
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    paused = False
    try:
        while i < n_frames_total-1:
            # 非阻塞读取按键
            dr, _, _ = select.select([sys.stdin], [], [], 0.0)
            if dr:
                ch = sys.stdin.read(1)
                if ch == " ":
                    paused = not paused
                    print(f"Paused: {paused}")
                elif ch in ("\x03", "q"):
                    print("Exit requested.")
                    break

            if paused:
                time.sleep(0.05)
                continue
            
            print(f"Processing frame {i+1}/{n_frames_total-1}")
            # FPS measurement
            fps_counter += 1
            current_time = time.time()
            if current_time - fps_start_time >= fps_display_interval:
                actual_fps = fps_counter / (current_time - fps_start_time)
                print(f"Actual rendering FPS: {actual_fps:.2f}")
                fps_counter = 0
                fps_start_time = current_time

            # Update task targets.
            lafan1_data = lafan1_data_frames[i]

            # visualize
            robot_motion_viewer.step(
                root_pos=root_pos[i],
                root_rot=root_rot[i],
                dof_pos=dof_pos[i],
                human_motion_data=retargeter.scaled_human_data,
                # human_motion_data=smplx_data,
                human_pos_offset=np.array([0.0, 0.0, 0.0]),
                show_human_body_name=False,
                rate_limit=args.rate_limit, # 这里恢复为原来的设置，速度为现实速度
            )

            i += 1

    finally:
        # 恢复终端设置
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)

    # root_rot[:, [0, 1, 2, 3]] = root_rot[:, [1, 2, 3, 0]]
    if args.save_path is not None:

        # Compute velocities
        dof_vel = torch.gradient(torch.from_numpy(dof_pos).float(), spacing=1/src_fps, dim=0)[0]
        body_lin_vel_w = torch.gradient(body_pos, spacing=1/src_fps, dim=0)[0]
        body_ang_vel_w = _so3_derivative(rotations=body_rot, dt=1/src_fps)

        motion_data = {
            "fps": src_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_names": dof_names,
            "body_names": body_names,
            # "link_body_list": body_names,
            "dof_positions": dof_pos,
            "dof_pos": dof_pos,
            "body_positions": body_pos,
            "local_body_pos": body_pos,
            "body_rotations": body_rot,
            
            # dataset for beyond mimic
            "joint_names": dof_names,
            "joint_pos": dof_pos,
            "joint_vel": dof_vel,
            "body_pos_w": root_pos,
            "body_quat_w": root_rot,
            "body_lin_vel_w": body_lin_vel_w,
            "body_ang_vel_w": body_ang_vel_w,
        }
        
        print("dof names:", motion_data["dof_names"])
        print("body names:", motion_data["body_names"])
        print("saving motion data...")
           
            
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
    
        base_name = os.path.splitext(os.path.basename(args.bvh_file))[0]
        base_no_ext = os.path.join(args.save_path, base_name)
        npz_path = base_no_ext + ".npz"
        pkl_path = base_no_ext + ".pkl"
        txt_path = base_no_ext + ".txt"
        npz_dir = os.path.dirname(npz_path)
        if npz_dir:
            os.makedirs(npz_dir, exist_ok=True)
        # 准备 numpy-compatible dict
        try:
            npz_dict = to_numpy_compatible(motion_data)
        except Exception as e:
            print(f"[ERROR] Converting to numpy-compatible failed for {npz_path}: {e}")
            npz_dict = {}
            
        if args.save_as_npz:
            # 1) 保存 npz
            try:
                np.savez_compressed(npz_path, **npz_dict)
                print(f"Saved to {npz_path}")
            except Exception as e:
                print(f"[ERROR] Saving .npz failed for {npz_path}: {e}")
                
        if args.save_as_pkl:
            # 2) 保存 pkl
            try:
                joblib.dump(npz_dict, pkl_path)
                print(f"Saved to {pkl_path}")
            except Exception as _e:
                print(f"[WARN] joblib dump failed for {pkl_path}: {_e}")

        if args.save_as_txt:
            # 3) 保存 txt
            try:
                with open(txt_path, "w") as f:
                    for k, v in npz_dict.items():
                        f.write(f"{k}: {v}\n")
                print(f"Saved to {txt_path}")
            except Exception as e:
                print(f"[ERROR] Saving .txt failed for {txt_path}: {e}")
                
        if args.save_as_csv:
            # 4) 保存 csv
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
                
                

    # Close progress bar
    print("joint names:", dof_names)
    print("data shape:", {k: np.shape(v) for k, v in npz_dict.items()})
    robot_motion_viewer.close()
       
