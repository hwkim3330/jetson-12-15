#!/usr/bin/env python3
"""
Create TensorRT Engine for YOLO
Optimized for Jetson Orin Nano

This script converts a YOLO PyTorch model to TensorRT engine
for optimal inference performance on Jetson.
"""

import os
import sys
import argparse


def create_yolo_engine(model_path: str, output_path: str, imgsz: int = 640,
                       half: bool = True, device: int = 0):
    """
    Convert YOLO model to TensorRT engine.

    Args:
        model_path: Path to .pt model file
        output_path: Path for output .engine file
        imgsz: Input image size
        half: Use FP16 precision
        device: CUDA device ID
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        print("Error: ultralytics not installed")
        print("Install with: pip3 install ultralytics")
        sys.exit(1)

    print(f"Loading model: {model_path}")
    model = YOLO(model_path)

    print(f"Exporting to TensorRT...")
    print(f"  Image size: {imgsz}")
    print(f"  Half precision (FP16): {half}")
    print(f"  Device: cuda:{device}")

    # Export to TensorRT
    model.export(
        format='engine',
        imgsz=imgsz,
        half=half,
        device=device,
        simplify=True,
        workspace=4,  # GB
    )

    # Find the generated engine file
    engine_file = model_path.replace('.pt', '.engine')
    if os.path.exists(engine_file):
        # Move to desired location
        if output_path != engine_file:
            os.rename(engine_file, output_path)
            engine_file = output_path
        print(f"\nTensorRT engine created: {engine_file}")
        print(f"Size: {os.path.getsize(engine_file) / 1024 / 1024:.1f} MB")
    else:
        print("Error: Engine file not created")
        sys.exit(1)

    return engine_file


def main():
    parser = argparse.ArgumentParser(description='Create TensorRT engine for YOLO')
    parser.add_argument('--model', '-m', type=str, default='yolov8n.pt',
                        help='Path to YOLO .pt model')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output engine file path')
    parser.add_argument('--imgsz', '-s', type=int, default=640,
                        help='Input image size')
    parser.add_argument('--fp32', action='store_true',
                        help='Use FP32 instead of FP16')
    parser.add_argument('--device', '-d', type=int, default=0,
                        help='CUDA device ID')

    args = parser.parse_args()

    # Default output path
    if args.output is None:
        base_name = os.path.splitext(os.path.basename(args.model))[0]
        args.output = f'/home/nvidia/ros2_ws/src/keti/robot_ai/models/{base_name}.engine'

    # Ensure output directory exists
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    # Check if model exists, download if needed
    if not os.path.exists(args.model):
        print(f"Model not found: {args.model}")
        if args.model in ['yolov8n.pt', 'yolov8s.pt', 'yolov8m.pt']:
            print("Downloading model...")
            from ultralytics import YOLO
            model = YOLO(args.model)  # This will download
            args.model = args.model  # Use the downloaded path
        else:
            print("Please provide a valid model path")
            sys.exit(1)

    # Create engine
    engine_path = create_yolo_engine(
        model_path=args.model,
        output_path=args.output,
        imgsz=args.imgsz,
        half=not args.fp32,
        device=args.device
    )

    print("\nUsage in ROS2:")
    print(f"  ros2 run robot_ai yolo_detector --ros-args -p model_path:={engine_path}")


if __name__ == '__main__':
    main()
