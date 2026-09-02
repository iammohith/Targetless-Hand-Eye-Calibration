#!/usr/bin/env python3
"""
Targetless Hand-to-Eye Robot Calibration Utility
Based on the MDPI paper: "Stereo-Based Single-Shot Hand-to-Eye Calibration for Robot Arms"

This script computes the rigid coordinate transformation (rotation + translation) 
between a camera's coordinate frame and a robot's base coordinate frame using only 
three non-collinear point measurements from a single-shot stereo capture and a pointer tool.

No chessboard grids, circles, or external calibration targets are required during 
this calibration phase. The operator simply defines three physical non-collinear 
features in the workspace, measures them relative to the camera frame (using 3D depth/stereo),
and touches off on them with a calibrated robotic pointing tool.

Author: Mohith Sai Gorla
License: MIT
"""

import sys
import json
import argparse
import numpy as np

def compute_orthonormal_frame(p1, p2, p3):
    """
    Computes an orthonormal coordinate system from three non-collinear points.
    
    p1: Origin of the frame (3D vector)
    p2: A point defining the x-axis direction (3D vector)
    p3: A point defining the plane of the y-axis (3D vector)
    
    Returns:
        T: 4x4 homogeneous transformation matrix
    """
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    p3 = np.array(p3, dtype=float)
    
    # 1. Compute displacement vectors
    v_x = p2 - p1
    v_y_prime = p3 - p1
    
    # Check for collinearity or invalid points
    cross_prod = np.cross(v_x, v_y_prime)
    cross_norm = np.linalg.norm(cross_prod)
    if cross_norm < 1e-6:
        raise ValueError("The three points are collinear or too close to each other! Calibration requires three non-collinear points.")
    
    # 2. Normalize first axis (x-axis)
    x_hat = v_x / np.linalg.norm(v_x)
    
    # 3. Compute z-axis (perpendicular to the plane of p1, p2, p3)
    z_hat = cross_prod / cross_norm
    
    # 4. Compute y-axis (perpendicular to x_hat and z_hat to complete right-handed frame)
    y_hat = np.cross(z_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)  # Defensive normalization
    
    # 5. Form the rotation matrix by stacking unit vectors as columns
    R = np.column_stack((x_hat, y_hat, z_hat))
    t = p1
    
    # Assemble homogeneous transform
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    
    return T

def calibrate_camera_to_robot(cam_points, robot_points):
    """
    Computes the 4x4 camera-to-robot transform T_C_R using 3 point pairs.
    
    cam_points: list/array of 3 points in camera frame [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3]]
    robot_points: list/array of 3 points in robot frame [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3]]
    
    Returns:
        T_C_R: 4x4 homogeneous transformation matrix
    """
    if len(cam_points) != 3 or len(robot_points) != 3:
        raise ValueError("Exactly three points are required for single-shot pointer calibration.")
        
    T_W_C = compute_orthonormal_frame(cam_points[0], cam_points[1], cam_points[2])
    T_W_R = compute_orthonormal_frame(robot_points[0], robot_points[1], robot_points[2])
    
    # Transformation Composition: T_W_R = T_C_R * T_W_C -> T_C_R = T_W_R * (T_W_C)^-1
    T_C_R = T_W_R @ np.linalg.inv(T_W_C)
    return T_C_R, T_W_C, T_W_R

def run_self_test():
    """
    Runs an internal validation using experimental data from the MDPI paper
    (ZED2i Stereo Camera and UR10e Robot Arm) to verify numerical accuracy.
    """
    print("=" * 60)
    print("RUNNING CALIBRATION SELF-TEST (MDPI EXPERIMENTAL DATA)")
    print("=" * 60)
    
    # Points 1, 2, and 4 from Table 2 of the paper serve as calibration points P1, P2, P3
    cam_points = [
        [-56.76, -105.9, 490.43],    # P1 (Origin)
        [216.03, -102.55, 481.86],  # P2 (X-axis)
        [-59.12, 88.58, 494.7]      # P3 (Y-plane direction)
    ]
    
    robot_points = [
        [825.0, -473.0, 13.0],      # P1
        [1055.53, -326.87, 13.0],   # P2
        [929.18, -637.29, 14.0]     # P3
    ]
    
    T_C_R, T_W_C, T_W_R = calibrate_camera_to_robot(cam_points, robot_points)
    
    print("\nCalculated T_C_R Transform:")
    print(np.array2string(T_C_R, precision=6, suppress_small=True))
    
    # Validation against test points
    print("\nVerification on Calibration Points:")
    for idx, (p_c, p_r_gt) in enumerate(zip(cam_points, robot_points)):
        p_c_hom = np.append(p_c, 1.0)
        p_r_pred = (T_C_R @ p_c_hom)[:3]
        error = np.linalg.norm(p_r_pred - p_r_gt)
        print(f"  Point {idx+1}: Cam {p_c} -> Robot Pred {p_r_pred} | GT {p_r_gt} | Error: {error:.4f} mm")
        
    # Validation on un-calibrated test point (Table 2 - Point 10)
    # Cam: [137.27, -25.48, 487.02], Robot GT: [1032, -434, 13] (Calculated in robot: [1031.55, -434.51, 12.4])
    p10_c = np.array([137.27, -25.48, 487.02])
    p10_r_gt = np.array([1032.0, -434.0, 13.0])
    p10_r_pred = (T_C_R @ np.append(p10_c, 1.0))[:3]
    error_p10 = np.linalg.norm(p10_r_pred - p10_r_gt)
    print(f"\nGeneralization Test on Point 10 (Not used in calibration):")
    print(f"  Cam {p10_c} -> Robot Pred {p10_r_pred} | GT {p10_r_gt} | Error: {error_p10:.4f} mm")
    
    assert error_p10 < 1.5, "Self-test error exceeds acceptable threshold!"
    print("\n>>> SELF-TEST PASSED SUCCESSFULLY! Calibration math verified. <<<")
    print("=" * 60)

def main():
    parser = argparse.ArgumentParser(
        description="Calculate 3D Camera-to-Robot transform using targetless single-shot calibration."
    )
    parser.add_argument(
        "--test", action="store_true", help="Run internal self-test using paper's experimental data"
    )
    parser.add_argument(
        "--input", type=str, help="Path to JSON file containing camera and robot point sets"
    )
    parser.add_argument(
        "--out", type=str, help="Save computed transformation matrix to a JSON file"
    )
    
    args = parser.parse_args()
    
    if args.test:
        run_self_test()
        sys.exit(0)
        
    if not args.input:
        parser.print_help()
        print("\nNote: Use the --test flag to see the calibration utility work on real experimental data!")
        sys.exit(1)
        
    # Load point sets from input file
    try:
        with open(args.input, "r") as f:
            data = json.load(f)
            
        cam_pts = data["camera_points"]
        robot_pts = data["robot_points"]
    except Exception as e:
        print(f"Error loading point file: {e}")
        sys.exit(1)
        
    try:
        T_C_R, _, _ = calibrate_camera_to_robot(cam_pts, robot_pts)
        
        print("\n--- Calibration Succeeded! ---")
        print("Camera-to-Robot Transformation Matrix (T_C_R):")
        print(np.array2string(T_C_R, precision=6, suppress_small=True))
        
        # Extract Euler Angles (Z-Y-X rotation sequence, standard in robotics)
        R = T_C_R[0:3, 0:3]
        t = T_C_R[0:3, 3]
        
        # Pitch, Roll, Yaw extraction
        sy = np.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
            
        print(f"\nTranslation Vector t (mm): [X: {t[0]:.3f}, Y: {t[1]:.3f}, Z: {t[2]:.3f}]")
        print(f"Euler Angles (radians)  : [Roll (X): {x:.5f}, Pitch (Y): {y:.5f}, Yaw (Z): {z:.5f}]")
        print(f"Euler Angles (degrees)  : [Roll (X): {np.degrees(x):.3f}, Pitch (Y): {np.degrees(y):.3f}, Yaw (Z): {np.degrees(z):.3f}]")
        
        # Save output if requested
        if args.out:
            output_data = {
                "transformation_matrix": T_C_R.tolist(),
                "translation": t.tolist(),
                "euler_angles_deg": [np.degrees(x), np.degrees(y), np.degrees(z)]
            }
            with open(args.out, "w") as out_f:
                json.dump(output_data, out_f, indent=4)
            print(f"\nCalibration saved to: {args.out}")
            
    except Exception as e:
        print(f"\nCalibration Failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
