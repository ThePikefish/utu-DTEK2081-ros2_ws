import cv2
import numpy as np
import os

def apply_rgb_filters(image):
    """Apply RGB color filtering"""
    # TODO: (DONE) Define BGR color ranges for red, green, and blue
    # Example: ranges = {'red': ([low_B, low_G, low_R], [high_B, high_G, high_R]), ...}
    ranges = {
        'red': ([0, 0, 220], [50, 50, 255]),
        'green': ([0, 200, 0], [50, 255, 50]),
        'blue': ([220, 0, 0], [255, 80, 80]),
        'yellow': ([0, 220, 200], [10, 255, 255]),
        'purple': ([150, 0, 150], [250, 10, 190])
    }

    results = {}
    for color, (lower, upper) in ranges.items():
        # TODO: (DONE) Convert lists to NumPy arrays
        # TODO: (DONE) Create mask with cv2.inRange()
        # TODO: (DONE) Apply cv2.bitwise_and() to extract the color region
        lower_np = np.array(lower)
        upper_np = np.array(upper)
        mask = cv2.inRange(image, lower_np, upper_np)
        filtered = cv2.bitwise_and(image, image, mask=mask)  # Replace with masked image
        results[color] = {'mask': mask, 'filtered': filtered}
    
    return results

def apply_hsv_filters(image):
    """Apply HSV color filtering"""
    # TODO: (DONE) Convert image from BGR to HSV using cv2.cvtColor()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # TODO: Define HSV ranges for red, green, and blue
    # Hint: Red requires TWO ranges (low 0–10 and high 170–180)
    ranges = {
        'red': [
            ([0, 50, 10], [10, 255, 255]),    # low reds
            ([170, 50, 10], [180, 255, 255])  # high reds
        ],
        'green': [
            ([40, 10, 10], [86, 255, 255])
        ],
        'blue': [
            ([95, 10, 10], [126, 255, 255])
        ],
        'yellow': [
            ([28, 10, 10], [32, 255, 255])
        ],
        'purple': [
            ([140, 10, 10], [160, 255, 255])
        ]
    }

    results = {}
    for color, bounds_list in ranges.items():
        mask = None
        # TODO: (DONE) Convert lower/upper to NumPy arrays
        # TODO: (DONE) Create mask using cv2.inRange()
        # TODO: (DONE) Combine masks using cv2.bitwise_or()
        for lower, upper in bounds_list:
            lower_np = np.array(lower)
            upper_np = np.array(upper)

            new_mask = cv2.inRange(hsv, lower_np, upper_np)
            if mask is None:
                mask = new_mask
            else:
                mask = cv2.bitwise_or(mask, new_mask)
            pass
        
        # TODO: (DONE) Extract the color region using cv2.bitwise_and()
        filtered = cv2.bitwise_and(image, image, mask=mask)
        results[color] = {'mask': mask, 'filtered': filtered}
    
    return results

def display_results(original, rgb_results, hsv_results):
    """Display results in separate windows"""
    cv2.imshow('Original', original)
    
    for color in ['red', 'green', 'blue', 'yellow', 'purple']:
        # TODO: (DONE) Show the filtered images for both RGB and HSV
        # Hint: use cv2.imshow()
        cv2.imshow(f'RGB - {color}', rgb_results[color]['filtered'])
        cv2.imshow(f'HSV - {color}', hsv_results[color]['filtered'])
        pass

def main():
    """Main function"""
    # Load or create image
    image_path = "images/wheel.png"
    image = cv2.imread(image_path)
    print(f"Image loaded: {image.shape}")

    # TODO: (DONE) Apply both RGB and HSV filters
    rgb_results = apply_rgb_filters(image)
    hsv_results = apply_hsv_filters(image)

    # TODO: (DONE) Display results
    display_results(image, rgb_results, hsv_results)
       
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
