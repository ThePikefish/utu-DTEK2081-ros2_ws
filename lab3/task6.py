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
    # Hint: use np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='constant')
    pad_h = k_h // 2
    pad_w = k_w // 2
    padded = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='constant')

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
    image_path = "images/classroom.png"
    # Read image in grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError("Image file not found. Place 'sample.jpg' in the same folder.")
    
    # TODO: Define a simple 3x3 averaging kernel (all ones / 9)
    kernel = np.ones((3, 3), dtype=np.float32) / 9.0

    # Define a simple 3x3 sobel operators
    sobel_x = np.array([[-1, 0, 1],
                         [-2, 0, 2],
                         [-1, 0, 1]], dtype=np.float32)

    sobel_y = np.array([[-1, -2, -1],
                         [0, 0, 0],
                         [1, 2, 1]], dtype=np.float32)

    # Apply convolution 
    # (IN REALITY IS CROSS CORRELATION, we are not flipping the kernel)
    grad_x = convolve(image, sobel_x)
    grad_y = convolve(image, sobel_y)

    edges = np.sqrt(grad_x.astype(np.float32)**2 + grad_y.astype(np.float32)**2)
    edges = np.clip(edges, 0, 255).astype(np.uint8)

    edges = cv2.normalize(edges, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


    # TODO: Display results (original and blurred)
    cv2.putText(image, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)
    cv2.putText(grad_x, "Sobel X", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)
    cv2.putText(grad_y, "Sobel Y", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)
    cv2.putText(edges, "Sobel Magnitude", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)

    top    = np.hstack((image, grad_x))
    bottom = np.hstack((grad_y, edges))
    grid   = np.vstack((top, bottom))
    cv2.imshow("Sobel Edge Detection Results", grid)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
