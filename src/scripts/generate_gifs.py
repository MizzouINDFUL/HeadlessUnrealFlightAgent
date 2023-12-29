import os
import imageio.v2 as imageio

# Specify your PNG directory here
base_dir = '../../bags/'

# Specify the subfolders to generate GIFs from
subfolders = ['rgb', 'gt_vs_decl']

# Get all directories in base_dir
dirs = sorted([d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))])

# Select the last directory
last_dir = os.path.join(base_dir, dirs[-1])

# Get all subdirectories in last_dir
subdirs = sorted([d for d in os.listdir(last_dir) if os.path.isdir(os.path.join(last_dir, d))])

# Go through each subdirectory
for subdir in subdirs:
    # Go through each subfolder
    for subfolder in subfolders:
        png_dir = os.path.join(last_dir, subdir, subfolder)
        images = []

        # Get all file names in directory
        file_names = sorted((fn for fn in os.listdir(png_dir) if fn.endswith('.png')))

        # Read each file and append it to images list
        for filename in file_names:
            images.append(imageio.imread(os.path.join(png_dir, filename)))

        # Save images as an animated gif
        imageio.mimsave(os.path.join(png_dir, 'animated.gif'), images)