import cv2
import numpy as np
import matplotlib.pyplot as plt


def main():
    image = np.array(
        [
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 1, 0, 0, 0],
            [0, 0, 1, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
        ],
        dtype=np.uint8,
    )
    print(image.shape)
    kernel = np.array(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ],
        dtype=np.uint8,
    )

    plt.subplot(2, 3, 1)
    plt.imshow(image, cmap="gray")
    plt.title("Original")

    plt.subplot(2, 3, 2)
    image_dilated = cv2.dilate(image, kernel, anchor=np.array([-1, -1]), iterations=1)
    plt.imshow(image_dilated, cmap="gray")
    plt.title("Dilated, anchor=(-1, -1)")

    plt.subplot(2, 3, 3)
    image_dilated = cv2.dilate(image, kernel, anchor=np.array([0, 0]), iterations=1)
    plt.imshow(image_dilated, cmap="gray")
    plt.title("Dilated, anchor=(0, 0)")

    plt.subplot(2, 3, 4)
    image_dilated = cv2.dilate(image, kernel, anchor=np.array([0, 2]), iterations=1)
    plt.imshow(image_dilated, cmap="gray")
    plt.title("Dilated, anchor=(0, 2)")

    plt.subplot(2, 3, 5)
    image_dilated = cv2.dilate(image, kernel, anchor=np.array([2, 0]), iterations=1)
    plt.imshow(image_dilated, cmap="gray")
    plt.title("Dilated, anchor=(2, 0)")

    plt.subplot(2, 3, 6)
    image_dilated = cv2.dilate(image, kernel, anchor=np.array([3, 0]), iterations=1)
    plt.imshow(image_dilated, cmap="gray")
    plt.title("Dilated, anchor=(3, 0)")

    plt.show()


if __name__ == "__main__":
    main()
