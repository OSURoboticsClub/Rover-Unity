import gsteam_image_capture
import time

def main():
    gsteam_image_capture.write_frame(
        "gripper_" + str(time.time()) + ".jpg",
        42074,
        includeHeading=False,
        includeScale=True,
        scalePortion=0.094,
        scaleSize=5.0
    )

if __name__ == "__main__":
    main()
