# Copyright Epic Games, Inc. All Rights Reserved.
import unreal
import time
import sys
import glob

life_idx = 0
rain_alpha = 0.0
max_rain = 9000000.0

if (len(sys.argv) > 1):
    life_idx = int(sys.argv[1])


if life_idx >= 1 and life_idx <= 100:
    life_group = (life_idx - 1) // 10
    rain_alpha = life_group / 10.0
    unreal.SystemLibrary.execute_console_command(None, f"py add_rain.py {rain_alpha}")
else:
    unreal.log_warning("Invalid life index. Must be between 1 and 100.")

curr_rain = rain_alpha * max_rain

print("life_idx: " + str(life_idx))
print("rain_alpha: " + str(rain_alpha))
print("rain: " + str(curr_rain))

num_lives = 1
num_frames = -1

overrideResX = -1
overrideResY = -1

args = unreal.SystemLibrary.get_command_line()
tokens, switches, params = unreal.SystemLibrary.parse_command_line(args);

unreal.log("tokens: " + str(tokens))
unreal.log("switches: " + str(switches))
unreal.log("params: " + str(params))

if "num_frames" in params:
    num_frames = int(params["num_frames"])
    unreal.log("num_frames: " + str(num_frames))
else:
    unreal.log("num_frames not found in params")

if "imgOutputX" in params:
    overrideResX = int(params["imgOutputX"])
    unreal.log("overrideResX: " + str(overrideResX))

if "imgOutputY" in params:
    overrideResY = int(params["imgOutputY"])
    unreal.log("overrideResY: " + str(overrideResY))

def OnJobFinishedCallback(params):
    unreal.log("job finished")

def OnShotFinishedCallback(params):
    for shot in params.shot_data:
        for passIdentifier in shot.render_pass_data:
            unreal.log("render pass: " + passIdentifier.name)
            for file in shot.render_pass_data[passIdentifier].file_paths:
                pass

def OnQueueFinished(params, success):
    subsystem = unreal.get_editor_subsystem(unreal.MoviePipelineQueueSubsystem)
    pipelineQueue = subsystem.get_queue()
    pipelineQueue.delete_all_jobs()
    
    unreal.log("MRQ SIM FINISHED")
    quit()

subsystem = unreal.get_editor_subsystem(unreal.MoviePipelineQueueSubsystem)
pipelineQueue = subsystem.get_queue()



les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
lvl_ref = unreal.SoftObjectPath(unreal.SystemLibrary.conv_soft_object_reference_to_string(les.get_current_level()).split(":")[0])

asset_reg = unreal.AssetRegistryHelpers.get_asset_registry()
config_asset = asset_reg.get_asset_by_object_path('/MindfulPlugin/MRQ/MindfulMRQConfig.MindfulMRQConfig')
config_ref = config_asset.get_asset()

if num_frames > 0:
    asset_reg.get_asset_by_object_path('/MindfulPlugin/MRQ/MindfulSceneCaptureLS.MindfulSceneCaptureLS').get_asset().set_playback_end(num_frames)

job = pipelineQueue.allocate_new_job(unreal.MoviePipelineExecutorJob)
job.job_name = "MRQ SIM"
job.map = lvl_ref
job.sequence = unreal.SoftObjectPath("/MindfulPlugin/MRQ/MindfulSceneCaptureLS.MindfulSceneCaptureLS")
job.set_configuration(config_ref)

outputSetting = job.get_configuration().find_or_add_setting_by_class(unreal.MoviePipelineOutputSetting)
outputSetting.flush_disk_writes_per_shot = True

if overrideResX > 0 and overrideResY > 0:
    outputSetting.output_resolution = unreal.IntPoint(overrideResX, overrideResY)

for _ in range(num_lives-1):
    pipelineQueue.duplicate_job(job)

for job in pipelineQueue.get_jobs():
        unreal.log("Validating job " + str(job))

SubsystemExecutor = unreal.MoviePipelinePIEExecutor()
SubsystemExecutor.on_executor_finished_delegate.add_callable_unique(OnQueueFinished)
SubsystemExecutor.on_individual_job_work_finished_delegate.add_callable_unique(OnJobFinishedCallback)
SubsystemExecutor.on_individual_shot_work_finished_delegate.add_callable_unique(OnShotFinishedCallback)

unreal.SystemLibrary.execute_console_command(None, "py clear_renders.py")
unreal.SystemLibrary.execute_console_command(None, "py set_stencil_colors.py")
time.sleep(3)
unreal.SystemLibrary.execute_console_command(None, "py add_rain.py " + str(curr_rain))

subsystem.render_queue_with_executor_instance(SubsystemExecutor)