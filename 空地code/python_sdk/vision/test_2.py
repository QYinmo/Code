import cv2
import numpy as np
from pathlib import Path

# --- Helper Function: Get Overexposed Mask ---


def get_overexposed_mask(img_bgr, high_thresh=225):
    """
    Identifies overexposed (very bright) areas in an image.

    Args:
        img_bgr (numpy.ndarray): Input image (BGR format).
        high_thresh (int): Gray level threshold (0-255). Pixels above this value are
                           considered overexposed.

    Returns:
        numpy.ndarray: Boolean mask where True indicates an overexposed area.
    """
    if img_bgr is None:
        return None

    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    overexposed_mask = (gray > high_thresh)
    return overexposed_mask

# --- Helper Function: Apply CLAHE to Color Image ---


def apply_clahe_color(img_bgr, clip_limit=2.0, tile_grid_size=(8, 8)):
    """
    Applies CLAHE to a color image in the Lab color space (L channel).
    """
    if img_bgr is None:
        return None

    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
    l_channel = lab[:, :, 0]
    a_channel = lab[:, :, 1]
    b_channel = lab[:, :, 2]

    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    enhanced_l_channel = clahe.apply(l_channel)

    merged_lab = cv2.merge([enhanced_l_channel, a_channel, b_channel])
    enhanced_color_img = cv2.cvtColor(merged_lab, cv2.COLOR_LAB2BGR)

    return enhanced_color_img

# --- Main Processing Function: Correct Reflectance then CLAHE ---


def correct_reflectance_then_clahe(
    image_path,
    output_path=None,
    # Threshold for detecting overexposure (on original image)
    overexposure_thresh=225,
    # Factor to reduce brightness in overexposed areas (<1.0)
    brightness_reduce_factor=0.7,
    clahe_clip_limit=2.5,             # CLAHE contrast limit
    clahe_tile_grid_size=(8, 8),      # CLAHE tile grid size
    # Color for visualizing corrected areas (B,G,R)
    highlight_color=(0, 0, 255),
    transparency=0.6                  # Transparency for visualization mask
):
    """
    Processes an image by first correcting overexposed/reflectance areas,
    and then applying CLAHE.

    Args:
        image_path (str): Full path to the input image.
        output_path (str, optional): Path to save the final processed image.
        overexposure_thresh (int): Gray level threshold for overexposure detection.
        brightness_reduce_factor (float): Factor to reduce brightness in overexposed regions.
        clahe_clip_limit (float): CLAHE contrast limit.
        clahe_tile_grid_size (tuple): CLAHE tile grid size.
        highlight_color (tuple): Color for the visualization mask.
        transparency (float): Transparency of the visualization mask.

    Returns:
        tuple: (original_img, reflectance_corrected_img, final_clahe_img, highlighted_final_img)
               Returns None, None, None, None if processing fails.
    """
    try:
        # 1. Read the original image (robust to non-ASCII paths)
        if not Path(image_path).exists():
            print(
                f"Warning: Image path '{image_path}' does not exist. Attempting np.fromfile.")
            img_data = np.fromfile(str(image_path), dtype=np.uint8)
            img_bgr = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        else:
            img_bgr = cv2.imread(image_path, cv2.IMREAD_COLOR)

        if img_bgr is None:
            print(
                f"Error: Could not read image '{image_path}'. Check path and file integrity.")
            return None, None, None, None

        original_img = img_bgr.copy()

        # --- Step 1: Identify and Correct Reflectance (Overexposed) Areas ---
        # Get the mask of overexposed areas on the ORIGINAL image
        overexposed_mask = get_overexposed_mask(
            original_img, overexposure_thresh)
        if overexposed_mask is None:
            print(f"Failed to get overexposed mask for '{image_path}'.")
            return None, None, None, None

        # Start with a copy of the original for correction
        reflectance_corrected_img = original_img.copy()

        # Convert to Lab to adjust brightness
        lab_reflectance_corrected = cv2.cvtColor(
            reflectance_corrected_img, cv2.COLOR_BGR2LAB)
        l_channel_corrected = lab_reflectance_corrected[:, :, 0].astype(
            np.float32)

        # Apply brightness reduction only to overexposed areas
        l_channel_corrected[overexposed_mask] *= brightness_reduce_factor
        l_channel_corrected = np.clip(l_channel_corrected, 0, 255).astype(
            np.uint8)  # Ensure values are in range

        lab_reflectance_corrected[:, :, 0] = l_channel_corrected
        reflectance_corrected_img = cv2.cvtColor(
            lab_reflectance_corrected, cv2.COLOR_LAB2BGR)

        # --- Step 2: Apply CLAHE to the reflectance-corrected image ---
        final_clahe_img = apply_clahe_color(
            reflectance_corrected_img,
            clahe_clip_limit,
            clahe_tile_grid_size
        )
        if final_clahe_img is None:
            print(f"CLAHE processing failed for '{image_path}'.")
            return None, None, None, None

        # --- Visualization of the corrected areas ---
        # Create a colored overlay based on the overexposed_mask (from original image)
        color_overlay = np.zeros_like(final_clahe_img, dtype=np.uint8)
        # Use the mask detected on original image
        color_overlay[overexposed_mask] = highlight_color

        # Blend the overlay onto the final CLAHE image
        highlighted_final_img = cv2.addWeighted(
            final_clahe_img, 1 - transparency, color_overlay, transparency, 0
        )

        # Save the final CLAHE image (without visualization overlay)
        if output_path:
            if Path(output_path).is_dir():
                file_name = Path(image_path).name
                final_output_path = Path(
                    output_path) / f"refl_clahe_{file_name}"
            else:
                final_output_path = Path(output_path)

            final_output_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(final_output_path), final_clahe_img)
            print(
                f"Image '{Path(image_path).name}' processed and saved to: {final_output_path}")

        return original_img, reflectance_corrected_img, final_clahe_img, highlighted_final_img

    except Exception as e:
        print(f"Error processing image '{image_path}': {e}")
        return None, None, None, None

# --- Batch Processing Function ---


def batch_process_reflectance_then_clahe(
    input_dir,
    output_dir,
    reflectance_params,
    clahe_params,
    visual_params
):
    """
    Batch processes images by first correcting reflectance, then applying CLAHE.
    """
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    supported_formats = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff')

    print(f"Starting batch processing in directory: {input_dir}")
    print(
        f"Reflectance Correction Params: thresh={reflectance_params['overexposure_thresh']}, factor={reflectance_params['brightness_reduce_factor']}")
    print(
        f"CLAHE Params: clip_limit={clahe_params['clip_limit']}, tile_grid_size={clahe_params['tile_grid_size']}")

    processed_count = 0
    skipped_count = 0

    for filename in os.listdir(input_dir):
        file_path = Path(input_dir) / filename
        if file_path.is_file() and filename.lower().endswith(supported_formats):
            output_file_name = f"refl_clahe_{filename}"  # Prefix output files
            output_file_path = Path(output_dir) / output_file_name

            original, reflectance_corrected, final_clahe, highlighted_final = correct_reflectance_then_clahe(
                image_path=str(file_path),
                output_path=str(output_file_path),
                overexposure_thresh=reflectance_params['overexposure_thresh'],
                brightness_reduce_factor=reflectance_params['brightness_reduce_factor'],
                clahe_clip_limit=clahe_params['clip_limit'],
                clahe_tile_grid_size=clahe_params['tile_grid_size'],
                highlight_color=visual_params['highlight_color'],
                transparency=visual_params['transparency']
            )

            if final_clahe is not None:
                processed_count += 1
            else:
                skipped_count += 1

    print("\n--- Reflectance Correction + CLAHE Batch Processing Complete ---")
    print(f"Successfully processed images: {processed_count}")
    print(f"Skipped files: {skipped_count}")

# --- Generic Image Display Function ---


def display_image_with_scaling(window_name, img, max_width=900, max_height=800):
    """
    Displays an image, scaling it down if it exceeds maximum display dimensions.
    """
    if img is None:
        print(f"Cannot display image '{window_name}': Image data is None.")
        return

    h, w = img.shape[:2]

    scale = 1.0
    if w > max_width:
        scale = min(scale, max_width / w)
    if h > max_height:
        scale = min(scale, max_height / h)

    if scale < 1.0:
        new_w, new_h = int(w * scale), int(h * scale)
        resized_img = cv2.resize(
            img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        print(f"Image '{window_name}' scaled to {new_w}x{new_h} for display.")
        cv2.imshow(window_name, resized_img)
    else:
        print(f"Image '{window_name}' displayed at original size {w}x{h}.")
        cv2.imshow(window_name, img)


# --- Usage Example ---
if __name__ == "__main__":
    # --- Parameter Configuration ---
    SINGLE_IMAGE_PATH = r"E:\yolov8\ultralytics-8.2.0\2025_for_train_2\train\images\capture_20250730_211906_236.jpg"
    INPUT_BATCH_DIR = r"E:\yolov8\ultralytics-8.2.0\2025_for_train_2\train\images"
    # New output folder
    OUTPUT_BATCH_DIR = r"E:\yolov8\ultralytics-8.2.0\2025_for_train_2\train\images_processed_refl_clahe"

    # Parameters for reflectance correction (applied first)
    REFLECTANCE_PARAMS = {
        # Pixels > 220 in gray are considered overexposed (from original img)
        'overexposure_thresh': 120,
        'brightness_reduce_factor': 0.6  # Reduce brightness by 40% (1.0 - 0.6)
    }

    # Parameters for CLAHE (applied second)
    CLAHE_PARAMS = {
        'clip_limit': 2.5,
        'tile_grid_size': (8, 8)
    }

    # Visualization parameters
    VISUALIZATION_PARAMS = {
        'highlight_color': (0, 255, 255),  # Yellow (B, G, R) for highlight
        'transparency': 0.6
    }

    MAX_SINGLE_DISPLAY_WIDTH = 900
    MAX_SINGLE_DISPLAY_HEIGHT = 800

    # --- Run Single Image Processing Example ---
    print("--- Single Image Reflectance Correction then CLAHE Example ---")
    original_img, reflectance_corrected_img, final_clahe_img, highlighted_final_img = correct_reflectance_then_clahe(
        image_path=SINGLE_IMAGE_PATH,
        overexposure_thresh=REFLECTANCE_PARAMS['overexposure_thresh'],
        brightness_reduce_factor=REFLECTANCE_PARAMS['brightness_reduce_factor'],
        clahe_clip_limit=CLAHE_PARAMS['clip_limit'],
        clahe_tile_grid_size=CLAHE_PARAMS['tile_grid_size'],
        highlight_color=VISUALIZATION_PARAMS['highlight_color'],
        transparency=VISUALIZATION_PARAMS['transparency']
    )

    if original_img is not None:
        # 1. Display Original Image
        display_image_with_scaling(
            "1. Original Image", original_img, MAX_SINGLE_DISPLAY_WIDTH, MAX_SINGLE_DISPLAY_HEIGHT)

        # 2. Display Image After Reflectance Correction (before CLAHE)
        display_image_with_scaling("2. After Reflectance Correction (Pre-CLAHE)",
                                   reflectance_corrected_img, MAX_SINGLE_DISPLAY_WIDTH, MAX_SINGLE_DISPLAY_HEIGHT)

        # 3. Display Final Image After CLAHE (and reflectance correction)
        display_image_with_scaling("3. Final Image (Reflectance Corrected + CLAHE)",
                                   final_clahe_img, MAX_SINGLE_DISPLAY_WIDTH, MAX_SINGLE_DISPLAY_HEIGHT)

        # 4. Display Final Image with Highlighted Corrected Areas
        display_image_with_scaling("4. Final Image with Highlighted Corrected Areas",
                                   highlighted_final_img, MAX_SINGLE_DISPLAY_WIDTH, MAX_SINGLE_DISPLAY_HEIGHT)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

    # --- Run Batch Image Processing Example ---
    print("\n--- Batch Image Reflectance Correction then CLAHE Example ---")
    batch_process_reflectance_then_clahe(
        input_dir=INPUT_BATCH_DIR,
        output_dir=OUTPUT_BATCH_DIR,
        reflectance_params=REFLECTANCE_PARAMS,
        clahe_params=CLAHE_PARAMS,
        visual_params=VISUALIZATION_PARAMS
    )

    print("\nAll image processing complete. Please check the output directory.")
