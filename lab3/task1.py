import cv2
import numpy as np
import datetime

def process_image():
    # TODO: (DONE) Load an image from file
    # Hint: Use cv2.imread()
    image_path = "images/drone_image.jpg"  # TODO: (DONE) You'll need to provide this
    img = cv2.imread(image_path)  # Replace with actual image loading
    
    if img is None:
        print("Error: Could not load image")
        return
    
    print(f"Image shape: {img.shape}")
    print(f"Image dtype: {img.dtype}")

    # Parameters
    # ------------
    # BGR colors
    bgr_blue = (255, 0, 0)
    bgr_green = (0, 255, 0)
    bgr_red = (0, 0, 255)

    # Fonts
    font_complex = cv2.FONT_HERSHEY_COMPLEX
    font_plain = cv2.FONT_HERSHEY_PLAIN

    # Date time
    dt = str(datetime.datetime.now())
    # ------------

    
    # TODO: (DONE) Draw a rectangle on the image
    # Hint: cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    cv2.rectangle(img, pt1=(50, 50), pt2=(100, 100), color=bgr_blue, thickness=5)
    
    # TODO: (DONE) Draw a circle on the image
    # Hint: cv2.circle(img, center, radius, color, thickness)
    cv2.circle(img, center=(200, 200), radius=50, color=bgr_blue, thickness=10)

    # Filled triangle
    triangle_points = np.array([[400, 300], [500, 400], [300, 400]])
    cv2.fillPoly(img, pts=[triangle_points], color=bgr_blue)
    
    # TODO: (DONE) Add text to the image
    # Hint: cv2.putText(img, text, position, font, scale, color, thickness)
    cv2.putText(img, text='Kristo Jonsson & Xiaoxia Liu', org=(500, 600), fontFace=font_complex, fontScale=1, color=bgr_green, thickness=2)

    # Date and time text
    cv2.putText(img, text=dt, org=(30, 30), fontFace=font_plain, fontScale=2, color=bgr_red, thickness=2)

    # TODO: (DONE) Display the image
    # Hint: cv2.imshow() and cv2.waitKey()
    cv2.imshow("Image", img)

    cv2.waitKey(0) # Wait indefinitely for keypress until save and close
    
    # TODO: (DONE) Save the processed image
    # Hint: cv2.imwrite()
    savedImage_path = "images/task1_processed.jpg"
    cv2.imwrite(filename=savedImage_path, img=img)

if __name__ == "__main__":
    process_image()