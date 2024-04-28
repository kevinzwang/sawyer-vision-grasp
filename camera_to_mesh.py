import pyrealsense2 as rs
import numpy as np
import trimesh
import cv2
import os
from time import sleep


from mobile_sam import sam_model_registry, SamAutomaticMaskGenerator
import torch
import matplotlib.pyplot as plt


################ IMAGE SEG ################ 

def show_anns(anns):
    if len(anns) == 0:
        return
    sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((sorted_anns[0]['segmentation'].shape[0], sorted_anns[0]['segmentation'].shape[1], 4))
    img[:,:,3] = 0
    for ann in sorted_anns:
        m = ann['segmentation']
        color_mask = np.concatenate([np.random.random(3), [0.35]])
        img[m] = color_mask
    ax.imshow(img)


def get_object_mask(color_image):
    model_type = "vit_t"
    sam_checkpoint = "./weight/mobile_sam.pt"

    device = "cuda" if torch.cuda.is_available() else "cpu"

    mobile_sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
    mobile_sam.to(device=device)
    mobile_sam.eval()

    mask_generator = SamAutomaticMaskGenerator(mobile_sam)


    masks = mask_generator.generate(color_image)
    
    plt.figure(figsize=(20,20))
    plt.imshow(color_image)
    show_anns(masks)
    plt.axis('off')
    plt.show() 


    """
    mask_generator = SamAutomaticMaskGenerator(
        model=sam,
        points_per_side=32,
        pred_iou_thresh=0.86,
        stability_score_thresh=0.92,
        crop_n_layers=1,
        crop_n_points_downscale_factor=2,
        min_mask_region_area=100,  # Requires open-cv to run post-processing
    )
    """

    # rect_mask = np.zeros(color_image.shape[:2], dtype="uint8")

    # x_buffer = 10
    # x_mid = color_image.shape[0]/2 
    # x_min = x_mid - x_buffer
    # x_max = x_mid + x_buffer
    
    # y_buffer = 10
    # y_mid = color_image.shape[1]/2 
    # y_min = y_mid - y_buffer
    # y_max = y_mid + y_buffer


    # cv2.rectangle(rect_mask, (x_min, y_max), (x_max, y_max), 255, -1)

    # masked = cv2.bitwise_and(color_image, color_image, mask=rect_mask)


    # plt.figure(figsize=(20,20))
    # plt.imshow(masked)

    # plt.axis('off')
    # plt.show() 



    # plt.figure(figsize=(20,20))
    # plt.imshow(color_image)
    # show_anns(masks)
    # plt.axis('off')
    # plt.show() 




################ REALSENSE CALIBRATION ################ 

# This alligns the camera and starts the pipeline. It also returns the camera's depth profile/parameters.
def start_camera():

    # Start the pipeline and config the frame rate, resolution, etc
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
    
    # Align the image. Might not need?
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # realsense calibration parameters
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_parameters = depth_profile.get_intrinsics()

    return pipeline, align, depth_profile, depth_parameters


################ PHOTO CAPTURE ################ 


# Takes photos using the depth camera. Then saves to disk. 
def get_depth_images(pipeline, save_dir, frame_id):
    
    os.makedirs(save_dir, exist_ok=True)
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    # Convert frames to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    #depth_image= cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=255/65535), cv2.COLORMAP_JET) ## IF we want to colorize

    color_image = np.asanyarray(color_frame.get_data())
    
    # File paths and then write images
    depth_file = os.path.join(save_dir, f'depth_{frame_id}.png')
    color_file = os.path.join(save_dir, f'color_{frame_id}.png')
    cv2.imwrite(depth_file, depth_image)
    cv2.imwrite(color_file, color_image)

    print(f"Depth, Color taken: {depth_file}, {color_file}")



################ MESH CREATION ################ 


# Creates point cloud using depth image and depth camera intrinsics
def create_point_cloud(color_image, depth_image, depth_parameters):

    # CAMERA FRAME
    fx, fy = depth_parameters.fx, depth_parameters.fy
    cx, cy = depth_parameters.ppx, depth_parameters.ppy
    height, width = depth_image.shape

    # Create a meshgrid of pixel coordinates
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # WORLD FRAME
    Z = depth_image / 1000.0  
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    # Filter out invalide points

    # COLOR VALID FUNCTION HERE
    # color_valid = True

    # ## BOUNDING BOX
    # X_buffer = 10
    # X_min = min(X) - X_buffer
    # X_max = max(X) + X_buffer
    
    # Y_buffer = 10
    # Y_min = min(Y) - Y_buffer
    # Y_max = max(Y) + Y_buffer

    max_depth = 1000
    valid_depth = (depth_image != 0) & (depth_image < max_depth)

    # valid_depth = (Z !=0) & (Z < max_depth) & (X_min < X < X_max) & (Y_min < Y < Y_max)
    valid_depth = np.ravel(valid_depth)
    Z = np.ravel(Z)[valid_depth]
    X = np.ravel(X)[valid_depth]
    Y = np.ravel(Y)[valid_depth]

    # Can add color to lidar if needed by making 6 if needed
    vertices = np.dstack((X, Y, Z)).reshape(-1,3)

    return vertices

# Creates mesh from the depth images by calling create point cloud. 
def create_mesh(save_dir, depth_parameters):

    vertices_list = []
    # point_cloud = trimesh.PointCloud([])

    for file in os.listdir(save_dir):
        
        # Get the color and depth images
        if file.startswith('depth'):
            depth_path = os.path.join(save_dir, file)

            ### CHECK THIS WITH CAMERA. 
            color_path = os.path.join(save_dir, "color_" + file.split("_")[1])

            depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            color_image = cv2.imread(color_path, cv2.IMREAD_UNCHANGED)
            
            # create the point cloud from depth image
            if depth_image is not None:
                vertices = create_point_cloud(color_image, depth_image, depth_parameters)
                vertices_list.append(vertices)
                
    point_cloud = trimesh.PointCloud(np.vstack(vertices))  

    # Generate mesh from point cloud
    point_cloud.export(os.path.join(save_dir, 'point_cloud.ply'))
    print("Point Cloud Saved")



def main():
    save_dir = './images'

    pipeline, align, depth_profile, depth_parameters = start_camera()

    try:
        
        ### MOVE ROBOT HERE! ###

        # Just a temp loop. Take photos at robot positions. 
        num_images = 1
        for i in range(num_images):
            sleep(2)
            get_depth_images(pipeline, save_dir, i)


        # print("photos taken")
        
        # After all photos taken mesh created
        create_mesh(save_dir, depth_parameters)

        #create_point_cloud = trimesh.load('./images/point_cloud.ply', force='mesh')
        #mesh.show()

    finally:
        # Stop Pipe
        pipeline.stop()

if __name__ == "__main__":
    main()



"""
mobilesam
sam meta facebook
"""
