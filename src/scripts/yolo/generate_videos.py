import os
import subprocess

# Get the current working directory (i.e., the session folder)
session_dir = os.getcwd()

# Define the subfolders to process
subfolders = ['rgb', 'gt_vs_decl', 'predictions', 'rgb_ground_truth']

# Iterate over the life folders
for life_folder in os.listdir(session_dir):
    life_path = os.path.join(session_dir, life_folder)
    
    # Skip files and proceed with directories only
    if not os.path.isdir(life_path):
        continue

    # Create a videos directory inside each life folder
    video_dir = os.path.join(life_path, 'videos')
    os.makedirs(video_dir, exist_ok=True)

    # Iterate over the subfolders
    for subfolder in subfolders:
        subfolder_path = os.path.join(life_path, subfolder)

        # Skip if the subfolder doesn't exist
        if not os.path.exists(subfolder_path):
            continue

        # Define the output video file path
        video_file = os.path.join(video_dir, f'{subfolder}.mp4')

        # Generate the video file using ffmpeg
        subprocess.run(['ffmpeg', '-framerate', '12', '-pattern_type', 'glob', '-i', f'{subfolder_path}/*.png', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', video_file])
