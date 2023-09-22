import usb.core
import usb.util

class PixelRing:
    TIMEOUT = 8000

    def __init__(self, dev):
        self.dev = dev

    def mono(self, color):
        self.write(1, [(color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, 0])
    
    def set_color(self, rgb=None, r=0, g=0, b=0):
        if rgb:
            self.mono(rgb)
        else:
            self.write(1, [r, g, b, 0])

    def off(self):
        self.mono(0)

    def listen(self, direction=None):
        self.write(2)

    wakeup = listen

    def speak(self):
        self.write(3)

    def think(self):
        self.write(4)

    wait = think

    def spin(self):
        self.write(5)

    def show(self, data):
        self.write(6, data)

    customize = show
        
    def set_brightness(self, brightness):
        self.write(0x20, [brightness])
    
    def set_color_palette(self, a, b):
        self.write(0x21, [(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF, 0, (b >> 16) & 0xFF, (b >> 8) & 0xFF, b & 0xFF, 0])

    def set_vad_led(self, state):
        self.write(0x22, [state])

    def set_volume(self, volume):
        self.write(0x23, [volume])

    def change_pattern(self, pattern=None):
        print('Not support to change pattern')

    def write(self, cmd, data=[0]):
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, 0x1C, data, self.TIMEOUT)

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)


def find(vid=0x2886, pid=0x0018):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return
    return PixelRing(dev)



if __name__ == '__main__':
    import time
    pixel_ring = find()
    r = 0xff0000
    g = 0x00ff00
    b = 0x0000ff
    bl = 0
    w = 0xffffff
    while True:
        try:
            pixel_ring.set_brightness(0x01) #set brightness to min
            pixel_ring.mono(r) # set all LED to color red
            time.sleep(3)
            pixel_ring.mono(b) # set all LED to color blue
            time.sleep(3)
            pixel_ring.mono(g) # set all LED to color green
            time.sleep(3)
            pixel_ring.set_color_palette(r, w) #set a color palette and use to colors in this case red and white
            pixel_ring.think() # think needs to come together with the color palette
            time.sleep(3)
            pixel_ring.set_color_palette(b, g) #set a color palette and use to colors in this case blue and green
            pixel_ring.think() # think needs to come together with the color palette
            time.sleep(3)
            pixel_ring.mono(bl) # set all LED to color black/"turn off"
            time.sleep(3)
        except KeyboardInterrupt:
            break

    pixel_ring.off()
