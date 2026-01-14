#!/usr/bin/env python3
"""
ORB Feature Extraction and Visualization for ACFR Marine Data

This script loads left camera images, extracts ORB features using ORB_SLAM3 parameters,
and displays them with interactive preview.
"""

import sys
import os
import cv2
import yaml
import numpy as np
from acfr_marine_utils import load_stereo_data, decode_bayer_image


def load_yaml_config(yaml_path: str) -> dict:
    """
    Load camera and ORB parameters from YAML file.
    
    Args:
        yaml_path: Path to YAML configuration file
        
    Returns:
        Dictionary containing configuration parameters
    """
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_path}")
    
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def create_orb_detector(config: dict):
    """
    Create OpenCV ORB detector with parameters from ORB_SLAM3 config.
    
    Args:
        config: Configuration dictionary from YAML file
        
    Returns:
        OpenCV ORB detector
    """
    nfeatures = config.get('ORBextractor', {}).get('nFeatures', 1000)
    scale_factor = config.get('ORBextractor', {}).get('scaleFactor', 1.2)
    nlevels = config.get('ORBextractor', {}).get('nLevels', 8)
    ini_th_fast = config.get('ORBextractor', {}).get('iniThFAST', 20)
    min_th_fast = config.get('ORBextractor', {}).get('minThFAST', 7)
    
    # OpenCV ORB parameters
    # Note: OpenCV ORB uses different parameter names
    # We'll use the closest equivalent
    orb = cv2.ORB_create(
        nfeatures=nfeatures,
        scaleFactor=scale_factor,
        nlevels=nlevels,
        edgeThreshold=31,  # Default OpenCV value
        firstLevel=0,
        WTA_K=2,
        scoreType=cv2.ORB_HARRIS_SCORE,
        patchSize=31,
        fastThreshold=ini_th_fast  # Using iniThFAST as primary threshold
    )
    
    return orb, {
        'nfeatures': nfeatures,
        'scaleFactor': scale_factor,
        'nlevels': nlevels,
        'iniThFAST': ini_th_fast,
        'minThFAST': min_th_fast
    }


def extract_and_display_features(image_path: str, orb, config: dict, 
                                  image_idx: int, total_images: int, 
                                  timestamp: float):
    """
    Extract ORB features from an image and display them.
    
    Args:
        image_path: Path to the image file
        orb: OpenCV ORB detector
        config: Configuration dictionary
        image_idx: Current image index (0-based)
        total_images: Total number of images
        timestamp: Image timestamp
    """
    # Load image
    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"Warning: Could not load image: {image_path}", file=sys.stderr)
        return False
    
    # Decode Bayer pattern if needed
    img, format_desc = decode_bayer_image(img)
    print(f"  Image format: {format_desc}")
    
    # Extract green channel and use as grayscale
    if len(img.shape) == 3:
        # Image is BGR (from OpenCV), green channel is index 1
        gray = img[:, :, 1].copy()  # Green channel
        # Ensure it's a proper 2D array (not a view)
        gray = np.ascontiguousarray(gray)
        print(f"  Using green channel from RGB image (shape: {gray.shape})")
    else:
        # Already grayscale, use as-is
        gray = np.ascontiguousarray(img)
        print(f"  Image is already grayscale (shape: {gray.shape})")
    
    # Ensure grayscale is uint8 and 2D
    if gray.dtype != np.uint8:
        gray = gray.astype(np.uint8)
    if len(gray.shape) != 2:
        # If somehow still 3D, take first channel
        gray = gray[:, :, 0] if len(gray.shape) == 3 else gray.squeeze()
    
    # Extract ORB features
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    
    print(f"  Extracted {len(keypoints)} ORB features")
    
    # Draw keypoints on image
    # Convert grayscale green channel back to BGR for display
    if len(img.shape) == 3:
        display_img = img.copy()
    else:
        display_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    
    # Ensure display image is 8-bit (drawKeypoints requires uint8)
    if display_img.dtype != np.uint8:
        # Normalize 16-bit to 8-bit
        if display_img.dtype == np.uint16:
            display_img = (display_img / 256).astype(np.uint8)
        else:
            # For other types, normalize to 0-255 range
            display_img = cv2.normalize(display_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    img_with_features = cv2.drawKeypoints(
        display_img, 
        keypoints, 
        None, 
        color=(0, 255, 0),  # Green
        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    )
    
    # Add text overlay
    info_text = [
        f"Image {image_idx + 1}/{total_images}",
        f"Features: {len(keypoints)}",
        f"Time: {timestamp:.3f}s",
        f"File: {os.path.basename(image_path)}"
    ]
    
    y_offset = 30
    for i, text in enumerate(info_text):
        cv2.putText(img_with_features, text, 
                   (10, y_offset + i * 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (255, 255, 255), 2)
        cv2.putText(img_with_features, text, 
                   (10, y_offset + i * 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (0, 0, 0), 1)  # Black outline
    
    # Display image
    window_name = f"ORB Features - Image {image_idx + 1}/{total_images}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    # Set window size to be larger (width, height)
    cv2.resizeWindow(window_name, 1920, 1080)
    cv2.imshow(window_name, img_with_features)
    
    print(f"  Press any key to continue, 'q' to quit")
    
    # Wait for key press
    key = cv2.waitKey(0) & 0xFF
    cv2.destroyWindow(window_name)
    
    if key == ord('q'):
        return False
    
    return True


def main():
    """Main function to extract and display ORB features."""
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <data_directory> <yaml_config> <vocab_file>")
        print("\nExample:")
        print(f"  {sys.argv[0]} /path/to/data /path/to/cheryl.yaml /path/to/ORBvoc.txt")
        sys.exit(1)
    
    data_dir = sys.argv[1]
    yaml_path = sys.argv[2]
    vocab_path = sys.argv[3]
    
    # Validate paths
    if not os.path.isdir(data_dir):
        print(f"Error: Directory does not exist: {data_dir}", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(yaml_path):
        print(f"Error: YAML file does not exist: {yaml_path}", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(vocab_path):
        print(f"Error: Vocabulary file does not exist: {vocab_path}", file=sys.stderr)
        sys.exit(1)
    
    try:
        # Load YAML configuration
        print(f"Loading configuration from: {yaml_path}")
        config = load_yaml_config(yaml_path)
        
        # Create ORB detector with ORB_SLAM3 parameters
        orb, orb_params = create_orb_detector(config)
        print(f"\nORB Parameters:")
        for key, value in orb_params.items():
            print(f"  {key}: {value}")
        
        # Initialize ORB_SLAM3 system (to validate config)
        print(f"\nInitializing ORB_SLAM3 system...")
        try:
            import orbslam3
            slam = orbslam3.System(vocab_path, yaml_path, orbslam3.Sensor.MONOCULAR)
            slam.set_use_viewer(False)  # No viewer needed for feature extraction
            slam.initialize()
            print("  ORB_SLAM3 system initialized successfully")
        except ImportError:
            print("  Warning: orbslam3 module not found. Continuing with OpenCV ORB only.")
            slam = None
        except Exception as e:
            print(f"  Warning: Could not initialize ORB_SLAM3: {e}")
            print("  Continuing with OpenCV ORB only.")
            slam = None
        
        # Load stereo data (we only use left images)
        print(f"\nLoading stereo data from: {data_dir}")
        left_filenames, right_filenames, timestamps = load_stereo_data(data_dir)
        
        print(f"\nLoaded {len(left_filenames)} left images")
        print("="*60)
        print("Starting feature extraction and visualization...")
        print("Press any key to move to next image, 'q' to quit")
        print("="*60)
        
        # Process each image
        for i in range(len(left_filenames)):
            print(f"\n[{i+1}/{len(left_filenames)}] Processing: {os.path.basename(left_filenames[i])}")
            
            # Extract and display features
            continue_processing = extract_and_display_features(
                left_filenames[i], 
                orb, 
                config, 
                i, 
                len(left_filenames),
                timestamps[i]
            )
            
            if not continue_processing:
                print("\nFeature extraction cancelled by user.")
                break
        
        # Cleanup
        if slam:
            slam.shutdown()
        cv2.destroyAllWindows()
        
        print("\n" + "="*60)
        print("Feature extraction complete.")
        print("="*60)
        
        return 0
        
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    sys.exit(main())
