import time
import vl53l5cx
import ST7789
import numpy
from PIL import Image, ImageFilter
from matplotlib import cm


def get_palette(name):
    arr = numpy.array(cm.get_cmap(name, 256).colors * 255).astype('uint8')
    arr = arr.reshape((16, 16, 4))
    arr = arr[:, :, 0:3]
    return arr.tobytes()


display = ST7789.ST7789(
    width=240,
    height=240,
    rotation=90,
    port=0,
    cs=ST7789.BG_SPI_CS_BACK,  # Otherwise it will block the sensor!
    dc=9,
    backlight=18,
    spi_speed_hz=80 * 1000 * 1000,
    offset_left=0,
    offset_top=0
)

pal = get_palette("plasma")

vl53 = vl53l5cx.VL53L5CX()
vl53.set_resolution(8 * 8)

# This is a visual demo, so prefer speed over accuracy
vl53.set_ranging_frequency_hz(15)
vl53.set_integration_time_ms(5)
vl53.start_ranging()


while True:
    if vl53.data_ready():
        data = vl53.get_data()
        arr = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8))).astype('float64')

        # Scale view relative to the furthest distance
        # distance = arr.max()

        # Scale view to a fixed distance
        distance = 512

        # Scale and clip the result to 0-255
        arr *= (255.0 / distance)
        arr = numpy.clip(arr, 0, 255)

        # Invert the array : 0 - 255 becomes 255 - 0
        arr *= -1
        arr += 255.0

        # Force to int
        arr = arr.astype('uint8')

        # Convert to a palette type image
        img = Image.frombytes("P", (8, 8), arr)
        img.putpalette(pal)
        img = img.convert("RGB")
        img = img.resize((240, 240), resample=Image.NEAREST)
        # img = img.filter(ImageFilter.DETAIL)

        # Display the result
        display.display(img)

    time.sleep(0.01) # Avoid polling *too* fast