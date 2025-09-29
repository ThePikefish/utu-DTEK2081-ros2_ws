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
    img_h, img_w = image.shape
    k_h, k_w = kernel.shape

    # TODO: Pad the image with zeros around the border
    # Hint: use np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='')
    # Choose the type of padding to work with using the mode parameter
    pad_h = k_h // 2
    pad_w = k_w // 2
    padded = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='reflect')  # Reflection (Mirror) padding

    # TODO: Create an empty output image
    # Hint: matrix the same size of the current image
    output = np.zeros((img_h, img_w), dtype=np.float32)

    # TODO: Loop over each pixel (y, x)
    # Extract the region of interest (ROI) from padded image
    # Multiply by kernel and sum up values
    # Assign to output[y, x]
    # Hint: np.sum(region * kernel)
    for y in range(img_h):
        for x in range(img_w):
            region = padded[y:y + k_h, x:x + k_w]
            output[y, x] = np.sum(region * kernel)
    

    # Clip values to 0–255
    output = np.clip(output, 0, 255).astype(np.uint8)

    return output  

def main():
    image_path = "images/flower.png"
    # Read image in grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError("Image file not found. Place 'sample.jpg' in the same folder.")
    
    # TODO: Define a simple 5x5 averaging kernel (all ones / 25)
    kernel5 = np.ones((5, 5), dtype=np.float32) / 25.0

    # Define a simple 3x3 averaging kernel (all ones / 9)
    kernel3 = np.ones((3, 3), dtype=np.float32) / 9.0

    # TODO: Apply convolution
    # (IN REALITY IS CROSS CORRELATION, we are not flipping the kernel)
    blurred3 = convolve(image, kernel3)
    blurred5 = convolve(image, kernel5)

    # TODO: Create gaussian kernel and apply to image
    g1d = cv2.getGaussianKernel(5, 1)              
    gaussian_kernel = (g1d @ g1d.T).astype(np.float32)  # Gaussian 5x5 with sigma=1
    gaussian_blur = convolve(image, gaussian_kernel)

    # TODO: Display results (original and blurred)
    cv2.putText(image, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)
    cv2.putText(blurred3, "Box Blur (Manual Convolution) 3x3", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)
    cv2.putText(blurred5, "Box Blur (Manual Convolution) 5x5", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)
    cv2.putText(gaussian_blur, "Gaussian Blur (Manual Convolution)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)

    top    = np.hstack((image, blurred3))
    bottom = np.hstack((blurred5, gaussian_blur))
    grid   = np.vstack((top, bottom))
    cv2.imshow("Convolution Results", grid)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
