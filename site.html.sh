#!/bin/sh

echo "
<head>
  <title>backlight-auto</title>
  <style>
    h1, p, div, body { font-size: 1.1em; font-family: monospace; }
  </style>
</head>
<body>
  <h1>backlight-auto</h1>
  <p>Tired of adjusting brightness going from place to place?</p>
  <p>Has the sun gone down and not realize you're burning out your eyes?</p>
  <p>Want to adjust certain monitor's brightness using certain video sources?</p>
  <p>Looking for a tool that has 2 dependencies and gets the job done?</p>
  <p>backlight-auto is here to help.</p>
  <a href="$(echo "data:application/zip;base64,$(zip -R - $(git ls-files) | base64 -w 0)")">Download</a>
  <hr/>
  <p>There are multiple ways to setup using backlight-auto, but the first step
  is to tell the program what your webcam sees as \"black\".</p>
  <p>To start, place your thumb over your webcam.</p>
  <p>Run \`backlight-auto --measure --path-dev-video /dev/video0\`.</p>
  <p>Take the number printed and pass it into backlight-auto when you run it normally.</p>
  <p>\`backlight-auto --min-stimulus-length 1.48554e+02 --path-dev-video /dev/video0 --path-backlight /sys/class/backlight/intel_backlight/\`</p>
  <p>You'll need to adjust the permissions of your backlight files or run the program as super user.</p>
  <p>To run it periodically, create a systemd file.</p>
  <p>I personally assign it to a function key and run it when needed.</p>
</body>
";
