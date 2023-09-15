# backlight-auto

Automatically adjusts backlights based on webcam video brightness.

# Usage

First, cover your webcam so we can measure the darkest "stimulus length".
For this, I use my thumb. Then run:

```
backlight-auto --measure \
  --path-dev-video /dev/video0 \
  --path-backlight /sys/class/backlight/intel_backlight/
```

Use the number output like this:

```
backlight-auto --min-stimulus-length 1.484547e+02 \
  --path-dev-video /dev/video0 \
  --path-backlight /sys/class/backlight/intel_backlight/
```

You will have to either change the file permissions on your backlight files,
or run the command as super user, in order to change the brightness.

# Dependencies

* video4linux2
* libyuv
* zig
