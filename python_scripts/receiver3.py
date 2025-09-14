import serial
import struct
import time
import numpy as np
from PIL import Image


def receive_data():
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=2000000,
        timeout=1
    )

    try:
        while True:
            # Look for header
            header = bytearray()
            while len(header) < 4:
                byte = ser.read(1)
                if not byte:
                    continue

                header.append(byte[0])
                if len(header) == 4 and header != b'\xAA\xBB\xCC\xDD':
                    # Not a valid header, shift and continue
                    header = header[1:]

            print("Found header")

            size_bytes = ser.read(4)
            if not size_bytes or len(size_bytes) != 4:
                print("Failed to read size")
                continue

            image_size = struct.unpack("<I", size_bytes)[0]

            width_bytes = ser.read(4)
            if not width_bytes or len(width_bytes) != 4:
                print("Failed to read width")
                continue

            width = struct.unpack("<I", width_bytes)[0]

            height_bytes = ser.read(4)
            if not height_bytes or len(height_bytes) != 4:
                print("Failed to read height")
                continue

            height = struct.unpack("<I", height_bytes)[0]

            print(f"Expected image: {image_size} bytes, {width}x{height}")

            # read image data
            image_data = bytearray()
            bytes_received = 0


            ser.timeout = 5

            while bytes_received < image_size:
                chunk = ser.read(min(512, image_size - bytes_received))
                if not chunk:
                    print(f"Timeout after receiving {bytes_received}/{image_size} bytes")
                    break

                image_data.extend(chunk)
                bytes_received += len(chunk)
                print(f"Received {bytes_received}/{image_size} bytes", end="\r")

            print()

            # check if we received the expected amount of data
            if bytes_received != image_size:
                print(f"Incomplete image: got {bytes_received}/{image_size} bytes")
                continue

            # Look for footer
            footer = ser.read(4)
            if footer != b'\xDD\xCC\xBB\xAA':
                print(f"Invalid footer: {footer.hex()}")
                continue

            print("Valid footer received")

            # save
            try:
                # RGB565 2 bytes per pixel
                if width * height * 2 == image_size:
                    # cvt RGB565 to RGB888
                    rgb565_data = np.frombuffer(image_data, dtype=np.uint16)
                    rgb565_data = rgb565_data.byteswap()  # Swap byte order



                    r = ((rgb565_data & 0xF800) >> 11) << 3 | ((rgb565_data & 0xF800) >> 14)  # 5 bits -> 8 bits
                    g = ((rgb565_data & 0x07E0) >> 5) << 2 | ((rgb565_data & 0x07E0) >> 9)  # 6 bits -> 8 bits
                    b = (rgb565_data & 0x001F) << 3 | ((rgb565_data & 0x001F) >> 2)  # 5 bits -> 8 bits


                    img_array = np.stack([r, g, b], axis=-1).reshape((height, width, 3)).astype(np.uint8)
                    img = Image.fromarray(img_array)
                # RGB888
                elif width * height * 3 == image_size:
                    img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
                    img = Image.fromarray(img_array)
                # Grayscale format (1 byte per pixel)
                elif width * height == image_size:
                    img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))
                    img = Image.fromarray(img_array, mode='L')
                else:
                    print(f"Image dimensions ({width}x{height}) don't match data size ({image_size})")
                    continue

                timestamp = time.time()
                filename = f"saved_files/captured_image_{timestamp}.png"
                img.save(filename)
                print(f"Image saved to {filename}")

            except Exception as e:
                print(f"Error saving image: {e}")

            ser.timeout = 1

    except KeyboardInterrupt:
        print("\nStopping data reception")
    finally:
        ser.close()


if __name__ == "__main__":
    receive_data()