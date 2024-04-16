import unreal
import sys
import os
import glob

saved_folder = unreal.Paths.project_saved_dir()
movie_renders = glob.glob(saved_folder + "/MovieRenders/*")
for render in movie_renders:
    os.remove(render)