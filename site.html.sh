#!/bin/sh

echo "
<head>
  <title>backlight-auto</title>
  <style>
    body { max-width: 80ch; margin: auto; padding: 1em 1ch; }
    p, div, body { font-size: 1.1em; font-family: monospace; }
    body, a { background-color: black; color: lightgrey; }
    h1, h2, h3 { text-align: center; }
  </style>
</head>
<body>
  <h1>backlight-auto</h1>
  <h3>The missing backlight auto adjusting program for Linux.</h3>
  <p>Tired of adjusting brightness going from place to place?</p>
  <p>Has the sun gone down and not realize you're burning out your eyes?</p>
  <p>Want to adjust certain monitor's brightness using certain video sources?</p>
  <p>Looking for a tool that has 2 dependencies and gets the job done?</p>
  <p>backlight-auto is here to help.</p>
  <h2>Installation</h2>
  <p>Unfortunately backlight-auto is not yet packaged for distributions. If you're a package manager
  I'd love to help you package this.</p>
  <p>The first step is to install Zig.</p>
  <p>The second step is to install libyuv and video4linux2 headers.</p>
  <p>For both of those steps, please use a search engine. The instructions change all the time.</p>
  <p>Third, save this embedded zip (you've already downloaded it if you're viewing this page):</p>
  <center>
    <a href=\"$(echo "data:application/zip;base64,$(zip -r - $(git ls-files) .git | base64 -w 0)")\">download.zip</a>
  </center>
  <p>Last, build the project with \`zig build\` and copy \`zig-out/bin/backlight-auto\` to your preferred place.</p>
  <h2>Usage</h2>
  <p>There are multiple ways to setup using backlight-auto, but the first step
  is to tell the program what your webcam sees as \"black\".</p>
  <p>To start, place your thumb over your webcam.</p>
  <p>Run \`backlight-auto --measure --path-dev-video /dev/video0\`.</p>
  <p>Take the number printed and pass it into backlight-auto when you run it normally.</p>
  <p>\`backlight-auto --min-stimulus-length 1.48554e+02 --path-dev-video /dev/video0 --path-backlight /sys/class/backlight/intel_backlight/\`</p>
  <p>The program will output the recommended brightness value to be placed in the \`/sys/class/backlight/X/brightness\` file.</p>
  <p>I personally use the \`brightnessctl\` program, like so: \`brightnessctl s $(backlight-auto ...)\`.</p>
  <p>To run it periodically, create a systemd file.</p>
  <p>I personally assign it to a function key and run it when needed.</p>
  <h2>Information</h2>
  <p>The brightness calculation from the video input is based off the paper called \"Brightness Calculation in Digital Image Processing\" by Sergey Bezryadin, Pavel Bourov, and Dmitry Ilinih from January 2007.</p>
  <p>It takes by default 4 seconds to let the webcam \"get hot\", i.e. get proper exposure. This can be adjusted by passing \`--sample-time some-number\`.</p>
  <h2>Contributions</h2>
  <p>Email len@falken.directory with suggestions or patches.</p>
  <p>This tool is small, with the aim to be naturally forked and maintained forever.</p>
  <p>This software is evolutionary. The strongest fork is the official fork.</p>
</body>
";
