"""
Utility functions for loading and previewing ACFR Marine stereo camera data.

This module provides functions to:
- Load stereo image associations and timestamps from CSV files
- Preview stereo image pairs with Bayer pattern decoding
"""

import sys
import os
import csv
from pathlib import Path
from typing import Tuple, List, Optional
import cv2
import numpy as np


def find_image_file(data_dir: str, filename: str) -> Optional[str]:
    """
    Search for an image file in the data directory and its subdirectories.
    
    Args:
        data_dir: Root directory to search in
        filename: Name of the file to find
        
    Returns:
        Full path to the file if found, None otherwise
    """
    data_path = Path(data_dir)
    
    # First check root directory
    file_path = data_path / filename
    if file_path.exists():
        return str(file_path.resolve())
    
    # Search in subdirectories (excluding 'organized')
    for subdir in data_path.iterdir():
        if subdir.is_dir() and subdir.name != 'organized':
            file_path = subdir / filename
            if file_path.exists():
                return str(file_path.resolve())
    
    return None


def load_stereo_data(data_dir: str) -> Tuple[List[str], List[str], List[float]]:
    """
    Load stereo camera data from CSV files and prepare for ORB_SLAM3.
    
    Args:
        data_dir: Directory containing the data and organized/ subdirectory
        
    Returns:
        Tuple of (left_filenames, right_filenames, timestamps)
        - left_filenames: List of full paths to left (AC) images
        - right_filenames: List of full paths to right (FC) images
        - timestamps: List of timestamps in seconds
        
    Raises:
        FileNotFoundError: If CSV files don't exist
        ValueError: If CSV structure is invalid or data doesn't match
    """
    data_path = Path(data_dir)
    organized_dir = data_path / 'organized'
    
    # Check if organized directory exists
    if not organized_dir.exists():
        raise FileNotFoundError(f"Organized directory not found: {organized_dir}")
    
    name_mapping_file = organized_dir / 'sequence_name_mapping.csv'
    timestamp_mapping_file = organized_dir / 'sequence_timestamp_mapping.csv'
    
    # Check if CSV files exist
    if not name_mapping_file.exists():
        raise FileNotFoundError(f"Name mapping file not found: {name_mapping_file}")
    if not timestamp_mapping_file.exists():
        raise FileNotFoundError(f"Timestamp mapping file not found: {timestamp_mapping_file}")
    
    # Read name mapping CSV
    name_data = {}
    with open(name_mapping_file, 'r') as f:
        reader = csv.DictReader(f)
        if 'Sequence' not in reader.fieldnames or 'AC_Filename' not in reader.fieldnames or 'FC_Filename' not in reader.fieldnames:
            raise ValueError(f"Invalid CSV structure in {name_mapping_file}. Expected columns: Sequence, AC_Filename, FC_Filename")
        
        for row in reader:
            sequence = row['Sequence'].strip()
            ac_filename = row['AC_Filename'].strip()
            fc_filename = row['FC_Filename'].strip()
            name_data[sequence] = {
                'AC_Filename': ac_filename,
                'FC_Filename': fc_filename
            }
    
    # Read timestamp mapping CSV
    timestamp_data = {}
    with open(timestamp_mapping_file, 'r') as f:
        reader = csv.DictReader(f)
        if 'Sequence' not in reader.fieldnames or 'Timestamp' not in reader.fieldnames:
            raise ValueError(f"Invalid CSV structure in {timestamp_mapping_file}. Expected columns: Sequence, Timestamp")
        
        for row in reader:
            sequence = row['Sequence'].strip()
            try:
                timestamp = float(row['Timestamp'].strip())
            except ValueError:
                raise ValueError(f"Invalid timestamp value for sequence {sequence}: {row['Timestamp']}")
            timestamp_data[sequence] = timestamp
    
    # Merge data and validate
    sequences = sorted(name_data.keys())
    if set(sequences) != set(timestamp_data.keys()):
        missing_in_timestamps = set(sequences) - set(timestamp_data.keys())
        missing_in_names = set(timestamp_data.keys()) - set(sequences)
        error_msg = "Sequence mismatch between CSV files:\n"
        if missing_in_timestamps:
            error_msg += f"  Sequences in name mapping but not in timestamp mapping: {sorted(missing_in_timestamps)[:10]}...\n"
        if missing_in_names:
            error_msg += f"  Sequences in timestamp mapping but not in name mapping: {sorted(missing_in_names)[:10]}..."
        raise ValueError(error_msg)
    
    # Build file paths and timestamps
    left_filenames = []
    right_filenames = []
    timestamps = []
    missing_files = []
    
    for sequence in sequences:
        ac_filename = name_data[sequence]['AC_Filename']
        fc_filename = name_data[sequence]['FC_Filename']
        timestamp = timestamp_data[sequence]
        
        # Find image files
        left_path = find_image_file(data_dir, ac_filename)
        right_path = find_image_file(data_dir, fc_filename)
        
        if left_path is None:
            missing_files.append(f"Left (AC) image not found: {ac_filename}")
        if right_path is None:
            missing_files.append(f"Right (FC) image not found: {fc_filename}")
        
        if left_path and right_path:
            left_filenames.append(left_path)
            right_filenames.append(right_path)
            timestamps.append(timestamp)
        else:
            # Skip this sequence if files are missing
            print(f"Warning: Skipping sequence {sequence} due to missing image files", file=sys.stderr)
    
    if missing_files:
        print(f"Warning: {len(missing_files)} image files were not found. Some sequences may be skipped.", file=sys.stderr)
    
    if len(left_filenames) == 0:
        raise ValueError("No valid image pairs found. Check that image files exist in the data directory.")
    
    return left_filenames, right_filenames, timestamps


def decode_bayer_image(img: np.ndarray) -> Tuple[np.ndarray, str]:
    """
    Decode a Bayer pattern image (RGGB) to RGB.
    
    Args:
        img: Input image array (can be 2D, 3D with 1 channel, or 3D with 4 channels)
        
    Returns:
        Tuple of (decoded_image, format_description)
    """
    original_shape = img.shape
    
    if len(img.shape) == 2:
        # Single channel - decode Bayer pattern RGGB to BGR
        decoded = cv2.cvtColor(img, cv2.COLOR_BayerRGGB2BGR)
        desc = f"{original_shape} (Bayer) -> {decoded.shape} (RGB)"
    elif len(img.shape) == 3:
        if img.shape[2] == 1:
            # Single channel in 3D array
            decoded = cv2.cvtColor(img[:, :, 0], cv2.COLOR_BayerRGGB2BGR)
            desc = f"{original_shape} (Bayer) -> {decoded.shape} (RGB)"
        elif img.shape[2] == 4:
            # 4 channels - assume first channel contains Bayer pattern
            decoded = cv2.cvtColor(img[:, :, 0], cv2.COLOR_BayerRGGB2BGR)
            desc = f"{original_shape} (4-channel Bayer) -> {decoded.shape} (RGB)"
        elif img.shape[2] == 3:
            # Already RGB/BGR, no conversion needed
            decoded = img
            desc = f"{original_shape} (already RGB)"
        else:
            # Unknown format, return as-is
            decoded = img
            desc = f"{original_shape} (unknown format, no conversion)"
    else:
        # Unknown format, return as-is
        decoded = img
        desc = f"{original_shape} (unknown format, no conversion)"
    
    return decoded, desc


def preview_images(left_filenames: List[str], right_filenames: List[str], 
                   timestamps: List[float], num_preview: int = 5):
    """
    Preview the first N stereo image pairs side by side.
    
    Handles Bayer pattern (RGGB) images and decodes them to RGB for display.
    
    Args:
        left_filenames: List of paths to left (AC) images
        right_filenames: List of paths to right (FC) images
        timestamps: List of timestamps
        num_preview: Number of image pairs to preview (default: 5)
    """
    num_preview = min(num_preview, len(left_filenames))
    
    print(f"\nPreviewing first {num_preview} image pairs...")
    print("Press any key to move to the next image pair, or 'q' to quit preview.")
    
    for i in range(num_preview):
        # Load images as raw/unprocessed (to handle Bayer pattern)
        left_img = cv2.imread(left_filenames[i], cv2.IMREAD_UNCHANGED)
        right_img = cv2.imread(right_filenames[i], cv2.IMREAD_UNCHANGED)
        
        if left_img is None:
            print(f"Warning: Could not load left image: {left_filenames[i]}", file=sys.stderr)
            continue
        if right_img is None:
            print(f"Warning: Could not load right image: {right_filenames[i]}", file=sys.stderr)
            continue
        
        # Decode Bayer pattern if needed
        left_img, left_desc = decode_bayer_image(left_img)
        right_img, right_desc = decode_bayer_image(right_img)
        print(f"    Left image: {left_desc}")
        print(f"    Right image: {right_desc}")
        
        # Get image dimensions
        h1, w1 = left_img.shape[:2]
        h2, w2 = right_img.shape[:2]
        
        # Resize images to same height for side-by-side display
        target_height = max(h1, h2)
        if h1 != target_height:
            scale = target_height / h1
            new_width = int(w1 * scale)
            left_img = cv2.resize(left_img, (new_width, target_height))
        if h2 != target_height:
            scale = target_height / h2
            new_width = int(w2 * scale)
            right_img = cv2.resize(right_img, (new_width, target_height))
        
        # Concatenate images side by side
        combined = np.hstack([left_img, right_img])
        
        # Add text labels
        cv2.putText(combined, f"Left (AC) - {os.path.basename(left_filenames[i])}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, f"Right (FC) - {os.path.basename(right_filenames[i])}", 
                   (left_img.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, f"Sequence {i+1:05d} | Time: {timestamps[i]:.3f}s", 
                   (10, combined.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Display the combined image
        window_name = f"Stereo Pair {i+1}/{num_preview}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, combined)
        
        print(f"  Showing pair {i+1}/{num_preview}: {os.path.basename(left_filenames[i])} | {os.path.basename(right_filenames[i])}")
        
        # Wait for key press
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyWindow(window_name)
        
        if key == ord('q'):
            print("Preview cancelled by user.")
            break
    
    cv2.destroyAllWindows()
    print("Preview complete.")
