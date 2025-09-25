import cv2
import numpy as np
import os

def apply_rgb_filters(image):
    """Apply RGB color filtering"""
    # TODO: Define BGR color ranges for red, green, and blue
    # Example: ranges = {'red': ([low_B, low_G, low_R], [high_B, high_G, high_R]), ...}
    ranges = {}

    results = {}
    for color, (lower, upper) in ranges.items():
        # TODO: Convert lists to NumPy arrays
        # TODO: Create mask with cv2.inRange()
        # TODO: Apply cv2.bitwise_and() to extract the color region
        mask = None
        filtered = image  # Replace with masked image
        results[color] = {'mask': mask, 'filtered': filtered}
    
    return results

def apply_hsv_filters(image):
    """Apply HSV color filtering"""
    # TODO: Convert image from BGR to HSV using cv2.cvtColor()
    hsv = None

    # TODO: Define HSV ranges for red, green, and blue
    # Hint: Red requires TWO ranges (low 0–10 and high 170–180)
    ranges = {}

    results = {}
    for color, bounds_list in ranges.items():
        mask = None
        for lower, upper in bounds_list:
            # TODO: Convert lower/upper to NumPy arrays
            # TODO: Create mask using cv2.inRange()
            # TODO: Combine masks using cv2.bitwise_or()
            pass
        
        # TODO: Extract the color region using cv2.bitwise_and()
        filtered = image
        results[color] = {'mask': mask, 'filtered': filtered}
    
    return results

def display_results(original, rgb_results, hsv_results):
    """Display results in separate windows"""
    cv2.imshow('Original', original)
    
    for color in ['red', 'green', 'blue']:
        # TODO: Show the filtered images for both RGB and HSV
        # Hint: use cv2.imshow()
        pass

def main():
    """Main function"""
    # Load or create image
    image_path = "image.png"
    image = cv2.imread(image_path)
    print(f"Image loaded: {image.shape}")

    # TODO: Apply both RGB and HSV filters
    rgb_results = apply_rgb_filters(image)
    hsv_results = apply_hsv_filters(image)

    # TODO: Display results
    display_results(image, rgb_results, hsv_results)
       
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
