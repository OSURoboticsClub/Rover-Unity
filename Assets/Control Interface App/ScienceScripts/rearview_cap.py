import gsteam_image_capture
import time

def main():
    gsteam_image_capture.write_frame(
        "rearview_" + str(time.time()) + ".jpg",
        42071,
        headingOffset=180
    )

if __name__ == "__main__":
    main()
