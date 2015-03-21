# Robot WASD example

This example shows how to control the robot from within an android app. In
order to use this your phone has to support USB OTG (no root required).
Communication between the robot and the phone happens via the use of a USB
serial converter (FT232). To use the serial interface following library is
required:

<https://github.com/ksksue/FTDriver>

After installing the android application, connect the phone to the robot. The
application should open automatically. Connection has been established if the
textbox at the bottom contains the word "connected".

Inside the `AndroidManifest.xml` you will find and XML-node named
`intent-filter`. This entry states when the application will be launched. Using
this `intent-filter` option will automatically grant permission to communicate
over USB to the application. If you do not use this mechanism you have to
request them manually.

<http://developer.android.com/guide/topics/connectivity/usb/host.html#permission-d>
