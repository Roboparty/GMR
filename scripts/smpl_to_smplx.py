import os
import argparse
import numpy as np
from tqdm import tqdm

def convert_smpl_to_smplx(input_path, output_path, gender='neutral'):
    # Load SMPL data
    smpl_data = np.load(input_path, allow_pickle=True)
    data_dict = dict(smpl_data)  # Convert to dict for modification

    # 必要键检查：如果缺少任意一项则跳过
    has_mocap = ('mocap_framerate' in data_dict) or ('mocap_frame_rate' in data_dict)
    required_missing = []
    if 'betas' not in data_dict:
        required_missing.append('betas')
    if not has_mocap:
        required_missing.append('mocap_framerate / mocap_frame_rate')
    if 'poses' not in data_dict:
        required_missing.append('poses')
    if 'gender' not in data_dict:
        required_missing.append('gender')

    if required_missing:
        print(f"######################################### Skipping {input_path}: missing keys {required_missing} #########################################")
        return
    
    # Handle betas padding for SMPL-X (pad from 10 to 16 if necessary)
    if 'betas' in data_dict:
        betas = data_dict['betas']
        if betas.shape == (10,):
            data_dict['betas'] = np.concatenate([betas, np.zeros(6, dtype=betas.dtype)])
            print(f"Padded betas from 10 to 16 for {input_path}")
        elif betas.shape not in [(16,), (1, 16)]:
            raise ValueError(f"Unexpected betas shape: {betas.shape}. Expected (10,), (16,), or (1,16) for padding to SMPL-X.")

    # Handle mocap_frame_rate variations
    if 'mocap_framerate' in data_dict:
        data_dict['mocap_frame_rate'] = data_dict.pop('mocap_framerate')
        print(f"Renamed 'mocap_framerate' to 'mocap_frame_rate' for {input_path}")

    if 'poses' not in data_dict:
        raise ValueError("Input file does not contain 'poses' key. Is this an SMPL file?")

    poses = data_dict['poses']
    if poses.shape[1] > 72:
        poses = poses[:, :72]

    # Map to SMPL-X format
    data_dict['root_orient'] = poses[:, :3]
    data_dict['pose_body'] = poses[:, 3:66]  # 21 joints x 3 = 63, ignoring SMPL hand poses

    # Ensure gender is set
    if 'gender' not in data_dict:
        data_dict['gender'] = np.array(gender)

    # Remove original poses key
    del data_dict['poses']

    # Save as SMPL-X npz
    np.savez(output_path, **data_dict)
    print(f"Converted {input_path} to {output_path}")

# def process_directory(src_folder, tgt_folder, gender='neutral'):
#     os.makedirs(tgt_folder, exist_ok=True)
#     for filename in tqdm(os.listdir(src_folder)):
#         if filename.endswith('.npz'):
#             input_path = os.path.join(src_folder, filename)
#             output_path = os.path.join(tgt_folder, filename)
#             convert_smpl_to_smplx(input_path, output_path, gender)


def process_directory(src_folder, tgt_folder, gender='neutral'):
    os.makedirs(tgt_folder, exist_ok=True)
    for root, dirs, files in os.walk(src_folder):
        for filename in tqdm(files):
            if filename.endswith('.npz'):
                input_path = os.path.join(root, filename)
                # 保持相对路径结构
                rel_path = os.path.relpath(root, src_folder)
                output_dir = os.path.join(tgt_folder, rel_path)
                os.makedirs(output_dir, exist_ok=True)
                # 去掉文件名中的 "_poses" 部分
                out_filename = filename.replace("_poses", "")
                output_path = os.path.join(output_dir, out_filename)
                convert_smpl_to_smplx(input_path, output_path, gender)
                
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert SMPL motion data to SMPL-X format.")
    parser.add_argument("--src_folder", type=str, 
                        default="/home/msi/Desktop/AMASS/raw/CMU",
                        help="Source directory of SMPL .npz files")
    
    parser.add_argument("--tgt_folder", type=str, 
                        default="/home/msi/Desktop/CMU",
                        help="Target directory for SMPL-X .npz files")
    
    parser.add_argument("--input_file", type=str, help="Single input SMPL .npz file")
    parser.add_argument("--output_file", type=str, help="Single output SMPL-X .npz file")
    parser.add_argument("--gender", type=str, default="neutral", choices=["male", "female", "neutral"],
                        help="Gender for SMPL-X model if not present in file.")
    args = parser.parse_args()

    if args.src_folder and args.tgt_folder:
        process_directory(args.src_folder, args.tgt_folder, args.gender)
    elif args.input_file and args.output_file:
        convert_smpl_to_smplx(args.input_file, args.output_file, args.gender)
    else:
        parser.print_help()
