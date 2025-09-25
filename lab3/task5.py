#!/usr/bin/env python3
"""
Task 4: Convolution Blur - Student Template
Apply a blurring filter to an image using manual convolution
(without cv2.filter2D).
"""

import cv2
import numpy as np

def convolve(image, kernel):
    """Perform convolution on a grayscale image with a given kernel."""
    # TODO: Get image and kernel dimensions
    # Hint: .shape
    img_h, img_w = 0, 0
    k_h, k_w = 0, 0

    # TODO: Pad the image with zeros around the border
    # Hint: use np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='')
    # Choose the type of padding to work with using the mode parameter
    pad_h = k_h // 2
    pad_w = k_w // 2
    padded = None

    # TODO: Create an empty output image
    # Hint: matrix the same size of the current image
    output = None

    # TODO: Loop over each pixel (y, x)
    # Extract the region of interest (ROI) from padded image
    # Multiply by kernel and sum up values
    # Assign to output[y, x]
    # Hint: np.sum(region * kernel)

    # Clip values to 0–255
    output = np.clip(output, 0, 255).astype(np.uint8)

    return output  

def main():
    image_path = "flower.png"
    # Read image in grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError("Image file not found. Place 'sample.jpg' in the same folder.")
    
    # TODO: Define a simple 5x5 averaging kernel (all ones / 9)
    kernel = None

    # TODO: Apply convolution
    # (IN REALITY IS CROSS CORRELATION, we are not flipping the kernel)
    blurred = convolve(image, kernel)

    # TODO: Create gasussian kernel and apply to image
    gaussian_kernel = None
    gaussina_blur = convolve(image, gaussian_kernel)

    # TODO: Display results (original and blurred)
    cv2.imshow("Original", image)
    cv2.imshow("Box Blur (Manual Convolution)", blurred)
    cv2.imshow("Gaussian Blur (Manual Convolution)", gaussina_blur)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
