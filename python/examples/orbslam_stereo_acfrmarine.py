#!/usr/bin/env python3
"""
ORB_SLAM3 Stereo Image Loader for ACFR Marine Data

This script loads stereo camera image associations and timestamps from CSV files
and prepares them for ORB_SLAM3 processing. It does NOT run ORB_SLAM3 itself.
"""

import sys
import os
from acfr_marine_utils import load_stereo_data, preview_images


def main():
    """Main function to load and display stereo data."""
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <data_directory>")
        print("\nExample:")
        print(f"  {sys.argv[0]} /path/to/data/directory")
        sys.exit(1)
    
    data_dir = sys.argv[1]
    
    if not os.path.isdir(data_dir):
        print(f"Error: Directory does not exist: {data_dir}", file=sys.stderr)
        sys.exit(1)
    
    try:
        print(f"Loading stereo data from: {data_dir}")
        left_filenames, right_filenames, timestamps = load_stereo_data(data_dir)
        
        # Print summary
        print("\n" + "="*60)
        print("Data Loaded Successfully")
        print("="*60)
        print(f"Number of image pairs: {len(left_filenames)}")
        print(f"Timestamp range: {min(timestamps):.3f} to {max(timestamps):.3f} seconds")
        print(f"Duration: {max(timestamps) - min(timestamps):.3f} seconds")
        if len(timestamps) > 1:
            avg_interval = (max(timestamps) - min(timestamps)) / (len(timestamps) - 1)
            print(f"Average frame interval: {avg_interval:.3f} seconds")
        print("\nFirst 5 image pairs:")
        for i in range(min(5, len(left_filenames))):
            print(f"  [{i}] Sequence {i+1:05d}")
            print(f"      Left:  {os.path.basename(left_filenames[i])}")
            print(f"      Right: {os.path.basename(right_filenames[i])}")
            print(f"      Time:  {timestamps[i]:.3f} s")
        print("\n" + "="*60)
        print("Data is ready for ORB_SLAM3 processing.")
        print("="*60)
        
        # Preview first 5 image pairs
        try:
            preview_images(left_filenames, right_filenames, timestamps, num_preview=5)
        except KeyboardInterrupt:
            print("\nPreview interrupted by user.")
        except Exception as e:
            print(f"\nWarning: Could not preview images: {e}", file=sys.stderr)
            print("Continuing without preview...")
        
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
