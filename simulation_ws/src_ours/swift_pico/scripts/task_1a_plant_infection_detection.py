import cv2
import numpy as np
import os
import argparse
import matplotlib
matplotlib.use('Agg')  # Use Agg backend for headless rendering
import matplotlib.pyplot as plt

class PlantInfectionDetector:
    def __init__(self):
        # Initialize ArUco dictionary and parameters (OpenCV 4.5.4 compatible)
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError as e:
            print(f"Error: OpenCV ArUco module not found. Ensure OpenCV version is 4.5.4 or compatible. {e}")
            raise

    def detect_aruco_markers(self, image):
        """Detect ArUco markers in the image - OpenCV 4.5.4 compatible"""
        try:
            if len(image.shape) == 3:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            corners, ids, _ = cv2.aruco.detectMarkers(image_rgb, self.aruco_dict, parameters=self.aruco_params)
            return image_rgb, corners, ids
        except Exception as e:
            print(f"Error in ArUco marker detection: {e}")
            return image, None, None

    def extract_roi(self, image, corners, ids):
        """Extract ROI using detected marker IDs"""
        if ids is None or len(ids) < 4:
            print("Error: Fewer than 4 ArUco markers detected.")
            return None, None

        try:
            detected_ids = ids.flatten()
            corner_points = []
            for marker_id in detected_ids:
                marker_index = np.where(ids.flatten() == marker_id)[0]
                if len(marker_index) > 0:
                    marker_corners = corners[marker_index[0]][0]
                    center = np.mean(marker_corners, axis=0)
                    corner_points.append(center)

            if len(corner_points) != 4:
                print("Error: Could not compute 4 corner points for ROI.")
                return None, None

            src_points = np.array(corner_points, dtype=np.float32)
            centroid = np.mean(src_points, axis=0)

            top_points = [p for p in src_points if p[1] < centroid[1]]
            bottom_points = [p for p in src_points if p[1] >= centroid[1]]

            top_points = sorted(top_points, key=lambda x: x[0])
            bottom_points = sorted(bottom_points, key=lambda x: x[0])

            ordered_points = np.array([
                top_points[0], top_points[1], bottom_points[1], bottom_points[0]
            ], dtype=np.float32)

            width, height = 800, 600
            dst_points = np.array([
                [0, 0], [width-1, 0], [width-1, height-1], [0, height-1]
            ], dtype=np.float32)

            matrix = cv2.getPerspectiveTransform(ordered_points, dst_points)
            roi = cv2.warpPerspective(image, matrix, (width, height))
            return roi, matrix
        except Exception as e:
            print(f"Error extracting ROI: {e}")
            return None, None

    def detect_plant_regions(self, block_image, block_name):
        """Detect plant regions using color and edge detection with watershed"""
        try:
            hsv = cv2.cvtColor(block_image, cv2.COLOR_RGB2HSV)
            lab = cv2.cvtColor(block_image, cv2.COLOR_RGB2LAB)

            green_lower = np.array([30, 30, 30])
            green_upper = np.array([85, 255, 255])
            green_mask = cv2.inRange(hsv, green_lower, green_upper)

            yellow_lower = np.array([15, 30, 30])
            yellow_upper = np.array([35, 255, 255])
            yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

            brightness_mask = cv2.inRange(lab[:, :, 0], 180, 255)
            plant_mask = cv2.bitwise_or(green_mask, yellow_mask)
            plant_mask = cv2.bitwise_or(plant_mask, brightness_mask)

            kernel = np.ones((5, 5), np.uint8)
            plant_mask = cv2.morphologyEx(plant_mask, cv2.MORPH_CLOSE, kernel)
            plant_mask = cv2.morphologyEx(plant_mask, cv2.MORPH_OPEN, kernel)

            sure_bg = cv2.dilate(plant_mask, kernel, iterations=3)
            dist_transform = cv2.distanceTransform(plant_mask, cv2.DIST_L2, 5)
            _, sure_fg = cv2.threshold(dist_transform, 0.3 * dist_transform.max(), 255, 0)
            sure_fg = np.uint8(sure_fg)
            unknown = cv2.subtract(sure_bg, sure_fg)

            _, markers = cv2.connectedComponents(sure_fg)
            markers = markers + 1
            markers[unknown == 255] = 0

            block_image_bgr = cv2.cvtColor(block_image, cv2.COLOR_RGB2BGR)
            markers = cv2.watershed(block_image_bgr, markers)

            unique_labels = np.unique(markers)
            plant_regions = []
            for label in unique_labels:
                if label <= 1 or label == -1:
                    continue
                target = (markers == label).astype(np.uint8) * 255
                contours, _ = cv2.findContours(target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    area = cv2.contourArea(contours[0])
                    if 300 < area < 30000:
                        x, y, w, h = cv2.boundingRect(contours[0])
                        plant_regions.append((x, y, w, h))

            if plant_regions:
                plant_regions = sorted(plant_regions, key=lambda r: (r[1], r[0]))
                if len(plant_regions) > 6:
                    plant_regions = sorted(plant_regions, key=lambda r: r[2] * r[3], reverse=True)[:6]
                    plant_regions = sorted(plant_regions, key=lambda r: (r[1], r[0]))

            if len(plant_regions) < 6:
                print(f"Using grid-based detection for {block_name} due to insufficient regions")
                plant_regions = self.detect_plants_by_grid(block_image)

            print(f"Found {len(plant_regions)} plant regions in {block_name}")
            return plant_regions
        except Exception as e:
            print(f"Error detecting plant regions in {block_name}: {e}")
            return []

    def detect_plants_by_grid(self, block_image):
        """Detect plants using a predefined 2x3 grid"""
        try:
            height, width = block_image.shape[:2]
            grid_cols, grid_rows = 2, 3
            col_width = width // grid_cols
            row_height = height // grid_rows
            margin_x = int(col_width * 0.1)
            margin_y = int(row_height * 0.1)
            plant_regions = []
            for row in range(grid_rows):
                for col in range(grid_cols):
                    x = col * col_width + margin_x
                    y = row * row_height + margin_y
                    w = col_width - 2 * margin_x
                    h = row_height - 2 * margin_y
                    plant_regions.append((x, y, w, h))
            return plant_regions
        except Exception as e:
            print(f"Error in grid-based detection: {e}")
            return []

    def detect_yellow_infections(self, plant_roi):
        """Detect yellow infections within a plant region"""
        try:
            hsv = cv2.cvtColor(plant_roi, cv2.COLOR_RGB2HSV)
            yellow_lower1 = np.array([20, 100, 100])
            yellow_upper1 = np.array([30, 255, 255])
            yellow_lower2 = np.array([15, 50, 50])
            yellow_upper2 = np.array([35, 255, 255])
            yellow_mask1 = cv2.inRange(hsv, yellow_lower1, yellow_upper1)
            yellow_mask2 = cv2.inRange(hsv, yellow_lower2, yellow_upper2)
            yellow_mask = cv2.bitwise_or(yellow_mask1, yellow_mask2)
            kernel = np.ones((3, 3), np.uint8)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
            total_pixels = plant_roi.shape[0] * plant_roi.shape[1]
            if total_pixels == 0:
                return False, 0.0
            yellow_pixels = np.sum(yellow_mask > 0)
            yellow_percentage = (yellow_pixels / total_pixels) * 100
            return yellow_percentage > 2, yellow_percentage
        except Exception as e:
            print(f"Error detecting yellow infections: {e}")
            return False, 0.0

    def crop_plant_blocks(self, roi_image):
        """Crop plant blocks from ROI"""
        try:
            if roi_image is None:
                return None, None, None
            height, width = roi_image.shape[:2]
            block1_y_start = int(height * 0.15)
            block1_y_end = int(height * 0.45)
            block2_y_start = int(height * 0.55)
            block2_y_end = int(height * 0.85)
            x_start = int(width * 0.5)
            x_end = int(width * 0.92)
            block1 = roi_image[block1_y_start:block1_y_end, x_start:x_end]
            block2 = roi_image[block2_y_start:block2_y_end, x_start:x_end]
            crop_coords = (block1_y_start, block1_y_end, block2_y_start, block2_y_end, x_start, x_end)
            return block1, block2, crop_coords
        except Exception as e:
            print(f"Error cropping plant blocks: {e}")
            return None, None, None

    def process_plant_block(self, block_image, block_name):
        """Process a plant block to detect infected plants"""
        try:
            plant_regions = self.detect_plant_regions(block_image, block_name)
            if len(plant_regions) < 6:
                print(f"Using grid-based detection for {block_name} as fallback")
                plant_regions = self.detect_plants_by_grid(block_image)
            infected_plant_ids = []
            result_image = block_image.copy()
            for i, (x, y, w, h) in enumerate(plant_regions):
                plant_roi = block_image[y:y+h, x:x+w]
                is_infected, yellow_percentage = self.detect_yellow_infections(plant_roi)
                plant_id = self.calculate_plant_id(i, block_name)
                color = (255, 0, 0) if is_infected else (0, 255, 0)  # Red for infected, green for healthy
                thickness = 3 if is_infected else 2
                cv2.rectangle(result_image, (x, y), (x + w, y + h), color, thickness)
                status = "INFECTED" if is_infected else "healthy"
                label = f"{plant_id} ({status})"
                cv2.putText(result_image, label, (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                if is_infected:
                    infected_plant_ids.append(plant_id)
                    print(f"  {plant_id}: {yellow_percentage:.1f}% yellow - INFECTED")
                else:
                    print(f"  {plant_id}: {yellow_percentage:.1f}% yellow - healthy")
            return infected_plant_ids, result_image
        except Exception as e:
            print(f"Error processing {block_name}: {e}")
            return [], block_image

    def calculate_plant_id(self, index, block_name):
        """Calculate plant ID based on index (2x3 grid)"""
        block1_plant_letters = ['D', 'E', 'F', 'B', 'A', 'C']  # Custom sequence for Block-1 (P1)
        block2_plant_letters = ['E', 'F', 'D', 'A', 'C', 'B']  # Custom sequence for Block-2 (P2)
        prefix = "P2" if block_name == "Block-1" else "P1"
        plant_letters = block1_plant_letters if block_name == "Block-1" else block2_plant_letters
        return f"{prefix}{plant_letters[index % 6]}"

    def visualize_processing_steps(self, original_image, roi_image, block1, block2, 
                                 result1, result2, crop_coords, corners, ids):
        """Visualize processing steps and save to file"""
        try:
            fig, axes = plt.subplots(2, 4, figsize=(20, 10))
            image_with_markers = original_image.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image_with_markers, corners, ids)
            axes[0, 0].imshow(image_with_markers)
            axes[0, 0].set_title(f'1. Original with Markers\nIDs: {ids.flatten() if ids is not None else "None"}')
            axes[0, 0].axis('off')

            axes[0, 1].imshow(roi_image if roi_image is not None else np.zeros((100, 100, 3)))
            axes[0, 1].set_title('2. Extracted ROI')
            axes[0, 1].axis('off')

            roi_with_blocks = roi_image.copy() if roi_image is not None else np.zeros((100, 100, 3))
            if crop_coords is not None:
                b1_ys, b1_ye, b2_ys, b2_ye, x_s, x_e = crop_coords
                cv2.rectangle(roi_with_blocks, (x_s, b1_ys), (x_e, b1_ye), (0, 255, 0), 3)
                cv2.rectangle(roi_with_blocks, (x_s, b2_ys), (x_e, b2_ye), (255, 0, 0), 3)
                cv2.putText(roi_with_blocks, "Block-2 (P2)", (x_s, b1_ys-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(roi_with_blocks, "Block-1 (P1)", (x_s, b2_ys-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            axes[0, 2].imshow(roi_with_blocks)
            axes[0, 2].set_title('3. ROI with Plant Blocks')
            axes[0, 2].axis('off')

            axes[0, 3].imshow(block1 if block1 is not None else np.zeros((100, 100, 3)))
            axes[0, 3].set_title('4. Plant Block 2 (P2)')
            axes[0, 3].axis('off')

            axes[1, 0].imshow(block2 if block2 is not None else np.zeros((100, 100, 3)))
            axes[1, 0].set_title('5. Plant Block 1 (P1)')
            axes[1, 0].axis('off')

            axes[1, 1].imshow(result1 if result1 is not None else np.zeros((100, 100, 3)))
            axes[1, 1].set_title('6. Block 2 - Detection\nRed=Infected Green=Healthy')
            axes[1, 1].axis('off')

            axes[1, 2].imshow(result2 if result2 is not None else np.zeros((100, 100, 3)))
            axes[1, 2].set_title('7. Block 1 - Detection\nRed=Infected Green=Healthy')
            axes[1, 2].axis('off')

            axes[1, 3].text(0.1, 0.9, "PROCESSING COMPLETE", fontsize=12, fontweight='bold')
            summary_text = "✓ ROI Extraction\n✓ Plant Region Detection\n✓ Yellow Infection Analysis\n\n6 plants per block\n(2 columns × 3 rows)"
            axes[1, 3].text(0.1, 0.6, summary_text, fontsize=10)
            axes[1, 3].axis('off')

            plt.tight_layout()
            plt.savefig("processing_steps.png", bbox_inches="tight")
            plt.close()
            print("Visualization saved to 'processing_steps.png'")
        except Exception as e:
            print(f"Error in visualization: {e}. Skipping visualization.")

    def process_image(self, image_path):
        """Main processing pipeline"""
        print("=" * 60)
        print("PLANT INFECTION DETECTION SYSTEM")
        print("=" * 60)
        print("Detecting 6 plants per block (2×3 grid)")
        print("Yellow plants = INFECTED")
        print("=" * 60)

        try:
            if not os.path.exists(image_path):
                print(f"Error: File '{image_path}' not found!")
                return [], None, [], []

            image = cv2.imread(image_path)
            if image is None:
                print(f"Error: Could not load image from '{image_path}'")
                return [], None, [], []

            print(f"Image loaded: {image.shape}")
            original_image, corners, ids = self.detect_aruco_markers(image)
            if ids is None:
                print("No ArUco markers detected!")
                return [], None, [], []

            marker_ids = ids.flatten()
            print(f"Detected ArUco marker IDs: {marker_ids}")
            roi_image, _ = self.extract_roi(original_image, corners, ids)
            if roi_image is None:
                print("Failed to extract ROI!")
                return [], None, [], []

            print(f"ROI extracted: {roi_image.shape}")
            block1, block2, crop_coords = self.crop_plant_blocks(roi_image)
            if block1 is None or block2 is None:
                print("Failed to crop plant blocks!")
                return [], None, [], []

            print(f"Block 1 shape: {block1.shape}")
            print(f"Block 2 shape: {block2.shape}")
            print(f"\nAnalyzing Block-1 (6 plants):")
            infected_b1, result_b1 = self.process_plant_block(block1, "Block-1")
            print(f"\nAnalyzing Block-2 (6 plants):")
            infected_b2, result_b2 = self.process_plant_block(block2, "Block-2")
            all_infected = infected_b1 + infected_b2

            print("\n" + "=" * 60)
            print("DETECTION RESULTS")
            print("=" * 60)
            if all_infected:
                print("INFECTED PLANTS (Yellow-colored):")
                for plant_id in sorted(all_infected):
                    print(f"  {plant_id}")
            else:
                print("No infected plants detected.")
            print(f"\nSummary:")
            print(f"  Block 1: {len(infected_b2)}/6 plants infected")
            print(f"  Block 2: {len(infected_b1)}/6 plants infected")
            print(f"  Total: {len(all_infected)}/12 plants infected")

            # Visualize processing steps (in try-except to avoid blocking output.txt)
            try:
                self.visualize_processing_steps(original_image, roi_image, block1, block2, 
                                               result_b1, result_b2, crop_coords, corners, ids)
            except Exception as e:
                print(f"Visualization failed: {e}. Continuing with output file generation.")

            return all_infected, marker_ids, infected_b1, infected_b2
        except Exception as e:
            print(f"Error in processing pipeline: {e}")
            return [], None, [], []

def main():
    parser = argparse.ArgumentParser(description='Plant Infection Detection')
    parser.add_argument('--image', type=str, required=True, help='Path to input image')
    args = parser.parse_args()

    try:
        detector = PlantInfectionDetector()
        infected_plants, marker_ids, infected_b1, infected_b2 = detector.process_image(args.image)
        with open("output.txt", "w") as f:
            f.write(f"Detected marker IDs: {marker_ids.tolist() if marker_ids is not None else 'None'}\n")
            f.write(f"Infected plant in Block 1: {', '.join(infected_b2) if infected_b2 else 'None'}\n")
            f.write(f"Infected plant in Block 2: {', '.join(infected_b1) if infected_b1 else 'None'}\n")
        print(f"\nResults saved to 'output.txt'")
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
