
                    CoolTerm

       Copyright (c) 2007-2022 Roger Meier,
              All rights reserved

         http://freeware.the-meiers.org



WHAT IS IT?
===========

CoolTerm is an easy-to-use terminal for communication with hardware connected to serial ports.

CoolTerm is a simple serial port terminal application (no terminal emulation) that is geared towards hobbyists and professionals with a need to exchange data with hardware connected to serial ports such as servo controllers, robotic kits, GPS receivers, microcontrollers, etc.
The features of CoolTerm include:
- Capability of multiple concurrent connections on separate serial ports.
- Display of received data in plain text or hexadecimal format.
- Display of received data as graphic chart.
- Special receive and transmit character handling.
- Sending data via keypresses as well as a "Send String" dialog in plain text or hexadecimal format.
- Sending data via copy-paste of text into the terminal window.
- Sending of text/binary files via dialog or drag-and-drop.
- Capability of capturing received data to text/binary files.
- Local echo of transmitted data.
- Local echo of received data (loop-back to sender).
- Hardware (CTS, DTR) and software flow control (XON).
- Visual line status indicators.
- Manual control of RTS, DTS, and serial BREAK.
- Configurable character, line, and packet delays.
- Support for transmit macros.
- Saving and retrieving connection options.
- Scripting.
- and more...



INSTALLATION
============
CoolTerm comes without an installer and can be placed anywhere on the hard-drive as long as the correct folder structure is maintained. I.e. the included "Libs" and "Resources" folders must reside in the same location as the CoolTerm application.


SYSTEM REQUREMENTS
==================
Please refer to the platform specific "*** System Requirements.txt" document included with the download


HOW TO USE IT
=============
Please refer to the built-in help.


COMMUNITY
=========
Freeware Forum: http://forums.the-meiers.org/viewforum.php?f=1



VERSION HISTORY
===============

v2.0.1: 12/20/2022
------------------

FEATURE ENHANCEMENTS
- The line mode command history is can now be saved in the connection settings. This option can be enabled via the Terminal Options.
- Updated code to automatically create the application data folder if it doesn't exist when saving recent files and last session information.
- Added handling of iotcl errors on macOS and Linux when attempting connect to virtual serial ports creates with socat, ble-serial, etc. This allows connecting to such ports but the selected options such as baudrate, etc., may not be valid (which might not matter).
- Changed the default for "Wait for Termination String" under the file capture options from true to false.
- Replaced ESC[;H with ESC[2J as the escape sequence used to clear screen.
- Added option to use either ASCII 3 (ETX) or ESC[;H to home the display (i.e. scroll it to the very top).
- Improved transmit macro code to honor aborting the current macro during a long <delay> statement.

UI IMPROVEMENTS
- Increased the clickable area for the quick menu in the lower left corner and changed GUI widget.
- A clicked LEDs no longer toggles its value if the mouse button is released outside the LED.
- Made TX, RTS, and DTR LED labels clickable. Clicking the corresponding labels now has the same effect as clicking the LEDs themselves.
- Improved responsiveness of handshake LEDs.
- The font size in plain text and hex view can now be changed by using the scroll wheel while pressing the CTRL key.
- Added context menu item to plain text and hex displays to more easily access font settings.
- Updated menu short cuts to toggle status line to also require pressing the Alt (or Option on macOS) key to allow CMD-SHIFT-R to be used to stop a file capture again.

CHARTING IMPROVEMENTS
- Pressing the OK button in Chart Properties and Trace Properties windows now immediately redraws the chart rather than waiting for a refresh triggered by new received data.
- The x-axis scaling method is now automatically set to "Auto" whenever the source for the x-axis values is changed.
- Whenever the source for the x-axis value is changed to one of the data columns, CoolTerm will update the X-axis label to the name of the trace associated with the selected data column.
- Improved drawing code when plotting against "Time of day" on the x-axis to skip horizontal lines when time stamps change from the end of the day to the beginning of the day.

OTHER IMPROVEMENTS
- Updated Python scripting example 1 to demonstrate the use of additional useful commands.
- [Python Scripting] Improved text encodings for sending and receiving text data in CoolTerm.py module.
- Added instructions on how to revert CoolTerm to applications defaults to the built-in help.
- Updated crash report to use mailto: instead of FormMail.
- [Linux] Building for 64-Bit ARM systems
- [Linux] Support for dark mode
- [Win] Building for 64-Bit ARM systems

BUG FIXES
- Previous sessions are now restored even after an improper termination of the app.
- CoolTerm no longer tries to open an send progress window when sending macros with a lot of text, to prevent a ThreadAccessingUI exception.
- [Win] Fixed bug that would freeze the terminal window in plain text mode when moving it between screens with different display scaling factors (e.g. moving from a HiDPI screen to a non-HiDPI screen).
- Fixed bug that caused the serial port info button to not be populated with information when the Connection Options window first opened.
- fixed bug that would overwrite some chart properties when the dialog opens.
- Fixed bug that would cause traces to be drawn incorrectly when plotting against "Time of day" on the x-axis when timestamps include milliseconds.
- [Win][Linux] fixed bug that would tell the user that the scripting sockets are already in use when a 2nd instance of CoolTerm is launched and scripting sockets are enabled.
- [Win][Linux] fixed bug that would make the built-in help view not accessible when opened from the Chart and Trace Properties windows.
- [macOS] added code to prevent being able to close Connection Options or Trace Properties windows while a color picker window is still open as that could otherwise crash the app.



2.0.0: 09/17/2022
-----------------

NEW FEATURES
- New chart display (similar to Arduino serial plotter, but with a lot more options)
- Configurable toolbar
- New application, document, toolbar, and LED icons (courtesy of Greg Willits)
- New UI for transmit macro management
- Support for delays, serial BREAK, and repetition to transmit macros
- Support for controlling DTR and RTS signals from transmit macros.
- Support for raw TCP connections
- Support for select transmit and receive ANSI Escape sequences
- Capability to include "Received" and "Sent" labels in capture files
- Updated Hex Viewer with capability to select and copy/paste hex data
- Completely overhauled built-in help (HTML stylization courtesy of Greg Willits)
- [Win] Support for dark mode on Windows systems.

IMPROVEMENTS, CHANGED FEATURES
- Moved option to show/hide toolbar from preferences to connection settings
- Updated method used to store command line history. The command line history is now only updated if the current command is not equal to the previous one. Empty lines (enter key only) are not recorded.
- Added capability to manually toggle break signal by clicking the TX LED or via the Connection/Toggle Break menu item.
- Replaced status label context menus with dedicated quick access menu located in the bottom left of the status bar.
- Changed "Capture Local Echo" option to "Capture transmit data" and made capturing of outgoing data independent of whether local echo is turned on in the terminal options.
- Improved responsiveness of RX and TX status indicator LEDs.
- Split "Ignore Receive Signal Framing Errors" into separate options to ignore serial break conditions and framing errors.
- Updated status bar label with information on the currently running macro. When a macro is running the status label will change the text color to blue.
- Added option to automatically start a macro when the terminal opens.
- Added "Wait for remote echo" option for throttling data transmission.
- Updated Hex viewer with the ability to vary the number of characters per row when the window width changes.
- The text selection is maintained across view modes (e.g. plain text and hex).
- Added an additional date/time format to the Preferences to provide an option to remove time zone information.
- CoolTerm now checks the availability of fonts specified in loaded settings before applying. If a font is not available on the user's system, the default font will be applied.
- CoolTerm now checks the existence of the capture path specified in loaded settings before applying. If a capture path does not exist, the default path will be applied.
- "Wait for Termination String" in the "Capture File Options" is now independent of time stamps. This option can now be used without adding timestamps to captured data.
- The status display now shows the byte counts with thousands separators.
- Added option to connection settings to keep the terminal window always on top of other windows.
- Added menu item to the File menu to reveal the location of the current settings file.
- CoolTerm now saves crash log information to the user's application data directory.
- [Mac] CoolTerm now alerts the user when it is being launched from the downloaded disk image.
- Added "Help..." buttons to various dialogs.
- Added feature to avoid plain text flicker for Linux platforms (such as e.g. Raspberry Pi) where the way the text display is updated with the receive buffer contents causes the display to be redrawn multiple times when new data is added. The caveat is that enabling this feature will cause certain special character handling features to not work properly, though.

BUG FIXES
- Fixed a bug that would prevent 'Combine Contiguous Carriage Returns' and 'Handle CR as real Carriage Return' from working properly when 'Format TAB separated text' is enabled.
- [Win] Fixed bug that would cause a crash when printing the contents of the plain text display.



1.9.1: 11/11/2021
-----------------
NEW/CHANGED FEATURES:
- Added Find menu item to find text in plain text view.
- Added preference option to use the Home/End/PageUp/PageDown keys to scroll the terminal window contents instead of sending the corresponding keypresses via the serial port.

IMPROVEMENTS:
- Added support for optional TX Macro labels. Labels can be added to macros by appending a TAB character followed by the macro label. When a label is specified, it will be displayed in the macros menu instead of the actual macro.
- [Linux] UI tweaks for better compatibility across different disros.

BUG FIXES AND STABILITY IMRPOVEMENTS:
- Fixed bug that would not properly save multiple macros to connection settings files on Windows systems.
- Fixed bug that omitted the serial port selection and window position when saving settings.
- Implemented workaround for localization bug that caused an InvalidArgumentException when calling DateTime.now on some international Windows 10 systems.
- Fixed bug that would prevent characters with hex or octal code greater than 127 (decimal) from being used in macros.


1.9.0: 05/31/2021
-----------------
NEW/CHANGED FEATURES:
- Added support for transmit macros.
- Added code to detect sudden removal of serial devices. When a port is currently open, it will be closed when the device suddenly disappears.
- Added code to detect re-appearance of serial devices.
- Added option to automatically reconnect to a re-appearing device has suddenly disappeared before while the port was open. If enabled, CoolTerm will attempt to re-open the port. The re-connect delay can be configured as well. This is useful when attached microcontrollers are being rebooted.
- The automatic re-connect function also includes the use case when connection settings with "Automatically connect on open" enabled are opened and the selected port is not immediately available. CoolTerm will now automatically connect once the port becomes available.
- Added special character handling option to combine contiguous CR characters in to a single CR.
- Added new special character handling option to treat CR characters as real carriage returns. Instead of interpreting CR as a new line character, CoolTerm will instead move the insertion point to the beginning of the current line and overwrite the existing line with newly received characters.
- Added a "Plain Text" option to the selection of capture formats. When "Plain Text" is selected, the "Special Character Handing" settings are applied to the received data before capturing to file.
- Added option to wait for line endings before adding timestamps to received data. This emulates the timestamp behavior of the Arduino serial monitor.
- Added menu item under the View menu to show the current receive buffer fill level.
- Added preference options to include the serial port selection and terminal window position and size when saving connection settings as default.

IMPROVEMENTS:
- Extended range for custom baudrate input field. It is now possible to specify custom baudrates up to 100,000,000 baud.
- Added code to force a GUI update at Connect/Disconnect events as well as when a CoolTerm window first opens and when connection settings are changed. This avoids delays for GUI updates when long GUI refresh intervals are selected.
- Made further improvements to how unavailable ports are handled. CoolTerm will now re-scan for available serial ports before trying to access the port with the specified name.
- It is now possible to paste directly into the terminal window when Line Mode is enabled.
- Updated behavior when pasting text into the terminal window. CoolTerm will now sound a system beep when pasting is attempted while the port is closed.
- Change the default for "Refresh Serial Ports when opening Options" from false to true.
- Revised code to reference serial port object by their name instead of their index in array of available ports since that can change when the user adds/removes serial devices.
- Updated warning dialog when connection settings with an unavailable port are loaded with the option to select a different port, select the default port, or cancel, instead of just selecting the default port.
- Changed the built-in help to use a HTMLViewer control and updated the help text with a hyperlinked table of contents.
- Added preference option to use the default web browser instead of the built-in HTML viewer to display the built-in help.
- [macOS] now building universal binaries for Intel and ARM based Macs.


1.8.0: 10/12/2020
-----------------
NEW/CHANGED FEATURES:
- Replaced "Relative Time" timestamp format with "Time + Millis". It is now possible to record current time stamps with millisecond resolution, and the relative timestamp format is no longer necessary.
- Renamed the "Absolute Date and Time" and "Absolute" timestamp formats to "Date and Time" and "Time", respectively.
- Added code to improve accuracy for millisecond resolution timestamps.
- Added option to add timestamps to received data displayed in the CoolTerm window. The timestamps are added to the actual data, i.e. they will also be present in hex view.
- [Mac][Win] Added Preferences setting for smart display pausing. If enabled, scrolling up automatically pauses the display. Scrolling to the bottom automatically unpauses the display.
- "PauseDisplay" is no longer a Connection Settings parameter.
- Added "PauseDisplay(ID as integer, Value as boolean)" script command.
- Added keyboard shortcuts to toggle RTS and DTR handshake lines.
- Added "Remote" menu that is visible when the Remote Control Socket is enabled via the preferences. The menu item indicates if the socket is connected or not. The "Disconnect" menu item can be used to close and active connection.
- Added context menu item to connection status label for an easy way to reveal the capture file.
- Added button to "Save as Default" message dialog to open the default file location.
- Selecting either the rated or maximum speed from the serial port information popup in the Serial Port Options, will set the baud rate to the selected value.
- [Win] Added SPACE and MARK parity options (not supported on macOS or Linux yet).
- [macOS] 32-Bit builds are no longer supported.

IMPROVEMENTS:
- Added "GetFrontmostWindow as integer" script command.
- [Mac] GUI tweak for better compatibility with macOS Big Sur.
- [Win] Improved mouse wheel scroll performance.
- Added better error handling when the remote control socket port or HTTP server port are already in use by another app.
- Updated code for line mode history so that its behavior is consistent with UNIX style consoles.
- Updated code to use use Serial Port API 2.0.

BUG FIXES AND STABILITY IMRPOVEMENTS:
- Fixed bug in the script engine that prevented serial port parameters from being updated when a script changes them via "SetParameter".
- [Linux] fixed a bug that could sometimes result in bytes being lost during plain text to hexadecimal conversion.


1.7.0: 05/17/2020
-----------------
NEW/CHANGED FEATURES:
- Added preference setting to disable the save prompt that appears when a terminal window with unsaved changes is closed.
- Added "Remove High Bit from 8-bit Characters" option under "Special Character Handling".
- Added context menu to port status labels for a quick way to change the baud rate from the main window.
- Added context menu to connection status labels for a quick way to pause/resume/stop file capture.
- Added RX/TX byte counts to connection status label.
- Added configurations options for the connection timer and RX/TX byte counts to Connection Settings (Misc. Settings)
- Replaced option to reduce refresh rate with new option to configure the refresh interval between 10...10,000ms.
- Obsoleted selection for UTF8 text encoding and added new option to select from a variety of text encodings.
- Updated local echo code for when large files are being sent. The local echo now updates after transmission of each transmit packet.
- Removed Font Settings from the Preferences dialog. Font settings are no longer stored as part of the application wide preferences.
- Added Font Settings to Connection Options to allow individual settings in different terminal windows. Font settings are now stored as part of connection settings.
- [Mac] Setting default for "Enable AppleScript" to true, for consistency when upgrading from previous versions of CoolTerm where AppleScript was always enabled.
- [Mac] Added the following AppleScript Commands:
   - ShowWindow(ID as integer)  // brings specified window to foreground
   - Print(ID as integer)  // prints the contents of the specified window
- [BETA]: Cross-platform scripting support via remote control socket. 

IMPROVEMENTS:
- Added colorization to status labels.
- Renamed "Ignore receive signal errors" to "Ignore receive signal framing errors" Changed the default for "Ignore Receive Signal Errors" to True.
- [Win] Added *.bin and *.hex extensions to default file type set for the "Send Textfile" dialog
- [Win] Added *.* to the "All Files" type set to ensure all files are displayed in the "Send Textfile" dialog
- Renamed "Send Textfile..." menu item to "Send Text/Binary File...".
- Renamed "Capture Textfile..." menu item to "Capture Text/Binary File...".
- Renamed "Capture Text Options" group to "Capture File Options".
- Renamed "Send Text Options" group to "Send File Options"
- Minor tweaks to Preferences and Connections Settings GUIs to improve look on Linux and Raspberry Pi.
- Disabling startup splash window to improve launch time.
- Updated built-in help

BUG FIXES AND STABILITY IMRPOVEMENTS:
- [Win] Improved autoscrolling code to avoid flickering and undesired shifts of the scroll position.


1.6.0: 05/19/2019 (exactly 10 years since the initial public release!)
-----------------
NEW/CHANGED FEATURES:
- Added "Open Recent" menu item. Recent items are saved between sessions.
- Added "Close All" menu to close all open window.
- Added capability to restore the previous sessions. If enabled via the Preferences, the current session is saved if CoolTerm is quit via File/Quit (macOS/Linux) or File/Exit (Windows). CoolTerm will attempt to restore all the open terminal windows from the previous session at the next start.
- Added context menu for Copy/Paste operations to plain text display.
- Replaced option to stop Autoscrolling with option to pause the display instead. When enabled, the display contents are not updated until pausing the display is disabled again. Opening and closing the port automatically un-pauses the display.
- Added baud rate selector in the connections to allow setting a custom baudrate. The custom baudrate can be saved as part of the connection settings.
- The font size for text input fields (line mode input field "Send String" input field) can now be set independently via the preferences.
- Added preferences option to automatically refresh the list of serial ports when opening the connection options.
- Added preferences option to check for development releases when checking for updates.
- [Mac] Added the following AppleScript commands:
       - WindowIDfromName(WindowName as string) as integer
       - SaveSetting(ID as integer, FilePath as String) as boolean
       - CoolTermVersion as string
       - RescanSerialPorts
       - SerialPortCount as integer
       - SerialPortName(SerialPortIndex as integer) as string
       - GetCurrentSerialPort(ID as integer) as integer
       - SetCurrentSerialPort(ID as integer, SerialPortIndex as integer) as boolean
       - GetParameter(ID as integer, ParameterName as string) as string
       - SetParameter(ID as integer, ParameterName as string, Value as string) as boolean
       - GetAllParameters(ID as integer) as string
- Added support for dark mode on platforms that support it (such as macOS Mojave).
- Supporting Raspberry Pi
- Default is now 64-bit for all platforms (except Raspberry Pi)

IMPROVEMENTS:
- File/Save menu item is now permanently enabled. Selecting File/Save when settings have not previously been saved will present the user with a "Save As..." dialog.
- Connection Settings saved as default no longer contain the selected serial port and window position. New windows using the default settings will select the first available port rather than trying to force a port that may or may not be available.
- The text and background color settings for the text display are now also applied ot the line mode input text field.

BUG FIXES AND STABILITY IMRPOVEMENTS:
- Fixed bug where the receive buffer size wouldn't be set upon loading of connection settings.
- Fixed bug that causes setting to not be properly loaded on Mac when starting CoolTerm for the command line with a settings file as argument or when starting CoolTerm by double-clicking a settings file or dragging it onto the CoolTerm icon.
- [Mac] Disabled splash window for macOS Sierra and newer


1.5.0: 01/07/2018
-----------------
NEW/CHANGED FEATURES:

- Added configuration option for software supported flow control.
- Added configuration option for blocking keystrokes while transmit data flow is halted.
- Added printing for plain text and hex views.
- Added proxy settings to preferences (for update check).
- Changed preferences window to multi-tab concept.
- Made formatting of captured hex data configurable.
- It is now possible to use the 'Wait for termination string' option when capturing in hex format as long as formatting of hex data is disabled.
- Added "Packet Delay" option, which allows inserting a delay after the transmission of each packet, the size of which can now be specified via the connection settings GUI.
- The "Line Delay" option now supports matching all of the specified delay characters in addition to matching any of them. This allows for entire strings to be specified as line delay characters.

IMPROVEMENTS:

- Changed text encoding in "Send String" window to use system default encoding.
- Improved GUI for flow control settings to prevent hardware and software flow control from being enabled at the same time.
- Enhanced the behavior of the TX LED. When flow control is enabled and transmission is halted by the remote target, the color of the TX LED is changed to red to indicate that no data is being sent (select platforms only).
- The reception of break signals or framing errors now flashes the RX LED red when "Ignore receive errors" is enabled (select platforms only).
- Changed code to prevent the transmit progress from being displayed when "loop back receive data" is enabled and larger amounts of data are being looped back.
- [Win/Linux] Changed behavior of modal windows such as connection options and send progress windows so that only the parent terminal window is blocked as opposed to all open windows.
- [Mac] Added the following ApplesScript command:
		LookAheadHex(ID as integer) as String
- [Mac] Removed the following AppleScript commands:
		Str2Hex(PlainStr as String) as String
		Hex2Str(HexStr as String) as String
- [Mac] Universal Builds for Mac are no longer supported.

BUG FIXES AND STABILITY IMRPOVEMENTS:

- [Mac] Disabled SmartQuotes, SmartDashes, and SmartTextReplacement in TextArea
- Added handling of OutOfMemoryExceptions in transmit thread
- revised code to exclude text input line used in "Line Mode" from text size changes.


1.4.7: 02/11/17
---------------
NEW/CHANGED FEATURES:
- Added option to specify a custom file name for auto capture files when "Append to auto capture file" is enabled.
- CoolTerm will now save default settings to the application data directory, regardless of where CoolTerm is installed.
These locations are as follows
          Mac: /Users/UserName/Library/Application Support/CoolTerm/
          Win: \Users\UserName\AppData\Roaming\CoolTerm\
          Linux: /home/UserName/CoolTerm/.
However, a default.stc file placed in the same location as the CoolTerm executable will take precedence over the one in the application data location. This is useful for portable installations of CoolTerm.
- baudrates.ini and ports.ini files can now also be placed in the application data directory (see platforms specific locations above). However, files placed in the same location as the CoolTerm executable will take precedence over the files placed in the application data directory. This is useful for portable installations of CoolTerm.
- Made text wrapping in plain text view a configurable option.
- Added option to format TAB separated data for the plain text display. If enabled, text will be aligned on a specified column width.
- Added option to handle a specified minimum number of consecutive received spaces for the ASCII display. If enabled, such occurrences will be replaced by a TAB character.
IMPROVEMENTS:
- Clicking 'Cancel' in the transmit progress window will now dismiss the window even if when transmission is halted by the target when flow control is enabled.
- NUL characters are now ignored in ASCII view mode if "Handle non-print characters" is disabled to prevent the ASCII viewer from behaving erratically on certain platforms.
- Changed code to optimize CPU consumption in plain text view mode.
- Changed the default name of new terminals from "CoolTerm" to "Untitled" to better conform with common practice.
- Changed Capture File Save dialog to use .txt as file extension by default.
- [Mac] Added code to prevent App Nap when CoolTerm is running in the background.
- [Mac] Changed encoding of strings returned by Apple Script from ASCII to the system default to ensure compatibility with the full 8-bit character set.
- [Mac] New AppleScript commands:
        - WriteHex(ID as integer, HexData as String)
	- ReadHex(ID as integer, NumChars as Integer) as String
	- ReadAllHex(ID as integer) as String
- [Mac] Deprecating the Str2Hex and Hex2Str AppleScript functions. Future versions of CoolTerm will not implement these functions anymore
- Added dialog to prompt for user's e-mail when sending crash reports.
BUG FIXES AND STABILITY IMRPOVEMENTS
- Changed code so that port enumeration continues if an exception occurs with a certain port during operation. This should allow all valid ports to be enumerated.
- improved code to properly handle multiple instances (Windows and Linux).
- improved code for line condensing in ASCII view mode
- improved code to properly handle initial instance when new connection settings are opened.
- Fixed bug that resets the formatting of the plain text view after clearing the data in the receive buffer.


1.4.6: 02/16/16
---------------
NEW/CHANGED FEATURES:
- Added the option to automatically start file capture upon loading connection settings.
- Added option to append new data to auto capture files.
- Added option to filter ANSI escape sequences. If this option is enabled, ANSI sequence characters will be filtered in ASCII view mode.
- Added preference option to enable condensing the line spaces in plain text view mode (not available in Universal Binary builds).
- [Win][Linux] Added code to ensure that only one instance of CoolTerm is running on the system at the same time
- Added code to ensure that a connection settings file can only be opened once.
- Added preference option to show or hide the toolbar.
- Added UTF-8 support for plain text view.
- Updated preferences dialog to display extended character set.
- [Win] Added support for AltGr key combinations on certain international keyboards.
- Added shortcut (Mac: CMD-I, Win/Linux: CTRL-I) to Connection/Options... menu.
- Changed relative timestamp format from HH:MM:SS:sss to HH:MM:SS.sss to be consistent with established timestamp conventions.
IMPROVEMENTS:
- Removed CTRL+Alt+S short cut for "File/Save as default" to avoid AltGr+S triggering the short cut.
- [Win][Linux] Changed code to use platform specific default text encoding for the plain text window to allow displaying bytes larger than 0x7f.
- UI Tweaks to harmonize the look of the GUI between platforms.
- Updated Help Text with instructions on how to remove the serial port name from settings files.
BUG FIXES AND STABILITY IMRPOVEMENTS
- [Linux] Updated AutoScroll code to behave the same as it does on Mac and Windows.
- Fixed bug that didn't properly update the text of the Connection/Connect_Disconnect menu when switching between terminal windows.
- [Mac] Fixed bug where the text encoding of strings returned from AppleScript commands was not defined.


1.4.5: 02/14/15
---------------
NEW/CHANGED FEATURES:
- Added new option to handle Form Feed (FF, ASCII code 12) characters. When enabled, the reception of a FF character will clear the receive buffer and thus clear the screen.
- Added new option to handle End Of Text (EOT, ASCII code 4) characters. Enabling this feature will prevent the display from updating until a EOT character is received, at which time the display is updated with the contents from the receive buffer.
- Added code to present the user with the option to select a serial port if a loaded settings file includes an empty string for the port name. This allows the creation of generic settings files. 
- [MAC] Additional retina support.
BUG FIXES AND STABILITY IMRPOVEMENTS
- Code improvement to avoid ThreadAccessingUIException

NOTE TO MAC USERS:
Version 1.4.x will be the last release of CoolTerm available as Universal Binary. Starting with version 1.5.x, CoolTerm will only be deployed for Intel based Macs.


1.4.4: 09/21/14
---------------
NEW/CHANGED FEATURES:
- Added feature that shows the path to the current capture file (if a capture is currently active) when the mouse is hovered over the serial port status label at the bottom left of the CoolTerm window.
- changed Enter key emulation settings to use popup menu and added the option use a custom sequence to emulate the enter key.
- Added additional font sizes to preferences dialog.
- Added option to reduce the terminal refresh rate to once per second in order to reduce the CPU load on systems where power consumption is critical.
- Added text the the built in help to explain that reducing the size of the receive buffer can be used to reduce CPU power consumption.
- [MAC] Compiling for Cocoa from now on (intel based Macs only). The universal binary still uses Carbon.
- [MAC] Added basic retina support.
IMPROVEMENTS
- CoolTerm now properly remembers the last used folders (individually) for opening connection settings, capturing to text files, as well as sending text files.
- Added DEL character (ASCII 127) to the routine that handles BS characters in ASCII view.
- optimized code to reduce CPU load while sending text files.
- stability improvement to the code of the circular receive buffer.
BUG FIXES AND STABILITY IMRPOVEMENTS
- fixed code that could cause extended ASCII characters to sometimes be incorrectly translated from hexadecimal format to plain text.


1.4.3: 09/02/2013
-----------------
NEW/CHANGED FEATURES:
- Added preference setting to select the date and time format used for timestamps. The user can select between the SQL Date/Time format (YYYY-MM-DD HH:MM:SS) or the format determined by the users local system settings.
- Added options to choose the timestamp type and modified the code to fix an bug where the millisecond count and the Date/Time timestamp could get out of sync. It is now only possible to capture timestamps with millisecond resolution when selecting the relative timestamp format.
BUG FIXES AND STABILITY IMPROVEMENTS:
- Made various stability improvements to the code that updates the GUI while data is being sent and received.


1.4.2: 02/17/2013
-----------------
NEW/CHANGED FEATURES:
- Added options to set the initial state of RTS and DTR when the serial port opens. These options will only have an effect if the respective hardware flow control is not enabled.
- Changed the behavior of capturing received data to file when timestamps are enabled. Instead of appending a time stamp at every occurrence of CR and/or LF characters by default, it is now possible to specify the termination string at which to split the data. Furthermore, CoolTerm will now wait until the specified termination string is received until the data is captured to file.
- Added option to convert received data to hex before capturing to a capture file.
- Enabled the "Close Window" short cut for the "Send String" window.
- Added Alt+ENTER short cut for toggling between ASCII and Hex mode in "Send String"
- Added Connection/Reset menu item to provide a way to reset the port and unlock it if XOFF is active.
- Changed the behavior of the text file capturing with regards to the state of the port. It is now possible to start capturing before the port is opened. In addition, capturing is no longer stopped automatically when the port is closed. This allows for capturing to remain enabled even when the connection state of the port changes. 
- Added code to allow canceling of a text file transmission in progress if XOFF is active.
- Added option to the connection settings (Misc. Options) to automatically close the serial port when the window is being closed without showing a warning if the port is still open.
- Updated the built-in help with information on all menu items.

BUG FIXES AND STABILITY IMPROVEMENTS:
- fixed bug that would cause a NilObjectException in the Xmit routine in cases where the serial port is still transmitting or is hung while the port goes out of scope. This hopefully fixes the crash bug some users have experienced on occasion.
- Fixed bug that caused the transmit line delay setting to not properly be saved to connection settings files.
- Made improvements to receive buffer code to avoid resource conflicts.
Windows only:
- Changed file association on windows to work even on systems where the current user may not have administrative privileges.

APPLESCRIPT:
- Changed AppleScript commands to use unique terminal window IDs instead of the names when addressing windows.
- Added the following AppleScript commands
	- WindowCount as integer
	- WindowID(Index as integer) as integer
	- WindowName(index as integer) as String
	- IndexOfWindowID(ID as integer) as integer
	- NewWindow as integer
	- Quit
	- ResetPort(ID as integer)
	- FlushPort(ID as integer)
	- BytesLeftToSend(ID as integer) as integer
	- LastErrorCode(ID as integer) as integer
	- GetCTS(ID as integer) as boolean
	- GetDSR(ID as integer) as boolean
	- GetDCD(ID as integer) as boolean
	- GetRI(ID as integer) as boolean
	- CaptureStart(ID as integer, FilePath as string) as boolean
	- CapturePause(ID as integer)
	- CaptureResume(ID as integer)
	- CaptureStop(ID as integer)
	- SendTextFile(ID as integer, FilePath as string) as boolean
	- SendBreak(ID as integer)
	- SetDTR(ID as integer, Value as boolean)
	- GetDTR(ID as integer) as boolean
	- SetRTS(ID as integer, Value as boolean)
	- GetRTS(ID as integer) as boolean
- Added code to suppress error messages when CoolTerm is controlled by an AppleScript.
- Updated AppleScriptReadme.txt to reflect latest changes.


1.4.1: 11/21/2011
-----------------
- Improved handling of exceptions if preferences or connection settings can not be read or written.
- Checking serial port for nil in the transmit thread to avoid NilObjectException exceptions. Not clear if this fixes the issue a few users have seen.
- Improved exception handling when serial port adapters are added or removed from the system while CoolTerm is running.
- Fixed a bug in the circular receive buffer that could cause an OutOfBoundsException when reading data.
- Improved handing of OutOfMemoryExceptions when attempting to set the receive buffer size to a value larger than the amount of memory available on the user's system.


1.4.0: 9/12/2011
----------------
NEW FEATURES:
- New Connection options window with multiple pages.
- The connection options now display port information for the selected port.
- It is now possible to change baudrate, byte format settings, and flow control settings while the port is open.
- Added the option to specify additional baud rates via a "baudrates.ini" file. E.g. any baud rates that are known to be supported by the hardware that are not listed in the popup menu in the connection settings dialog can be added to a "baudrate.ini" file that resides in the same directory as CoolTerm.
- Added the option to specify additional serial ports via a "ports.ini" file. E.g. any devices such as /dev/tty.xxx devices on OSX and Linux that CoolTerm can not enumerate can be added to a "ports.ini" file that resides in the same directory as CoolTerm.
- Added the option to add timestamps to data captured to text files.
- Added a keyboard shortcut to connect/disconnect.
- Added option to replace TAB key presses with a configurable number of spaces (default = 4).
- Added option to enable/disable capturing of local echo of transmitted data in capture files.
- Added an option to keep the capture file open while capturing is in progress (default) or close it after writing and re-opening when new data arrives. This allows other applications to read the capture file while capturing is in progress.
- Added status LEDs for TX and RX to indicate activity related to sending and receiving data.
- Added preferences option to disable all menu shortcuts (on Windows and Linux only) in order to allow sending CTRL characters via the terminal. On Mac, the keyboard shortcuts use the Command key and thus don't interfere with CTRL characters.
- [MAC] AppleScript BETA: Added basic AppleScript handling for the most important terminal operations such as loading a settings file, opening/closing ports, and reading/writing data. The AppleScript functionality, while included in CoolTerm 1.4.0, is currently in public BETA , to allow a broader audience to beta test this feature and provide feedback. Refer to the attached "AppleScript ReadMe.txt" file for more details.
- [LINUX] Making LINUX version (unsupported) available.

IMPROVEMENTS:
- Made significant improvements to the code that processes received data, including changing architecture of the receive buffer to a circular buffer to improve efficiency and stability.
- Made significant improvements to the code that transmits data, including changing the architecture of the data transmission to be more asynchronous in nature to improve the responsiveness of CoolTerm during transmission of large text files, particularly with XON/XOFF flow control enabled
- CoolTerm now opens a progress window whenever the length of the text to be transmitted exceeds a certain threshold, and not only when text files are sent. 
- Flow control settings are now displayed in the terminal window as part of the port configuration string. For XON/XOFF the state is displayed, i.e. XON or XOFF.
- Added error messages to alert the user of errors that occur while attempting to open the serial port.
- Added check to warn the user if multiple files are dropped onto the CoolTerm window.
- "Send String" windows can now be resized.
- It is now possible to send CTRL characters when the terminal is set to Line Mode.
- Improved code for Line Mode to ensure that a pressed key is captured even if the command line didn't have the focus.
- Changed behavior of the status LEDs to better reflect the state of the signals. A green LED turned on now means that a signal is "active", the LED turned off means that it is "inactive".
- Changed the default state of DTR when a port is opened to "active" to conform with common practice, with the exception of Windows platforms when DTR flow control is enabled, in which case the default is "inactive" in order to avoid serial port errors.
- Improved handling of file IO errors when sending textile or capturing received data to textiles.
- Improved handling of file IO errors when reading and writing settings files.
- Improved error reporting. Crash reports will now include information about all open terminals.
- Slight change to the behavior during setting the break signal in that no characters are being read from the receive buffer. Received characters will be read after the break signal has been cleared.

BUG FIXES:
- Fixed a bug that would show an error message when the user chooses cancel in the capture file save dialog.
- Fixed a bug that threw an exception when opening the connection settings on a system without serial ports installed.
- Fixed a bug the displayed an error message when the user cancelled out of the "Send Textfile" dialog.
- [WIN] Fixed a bug where the removal of a serial port adapter could cause an exception when starting a connection.
- [MAC] Implemented a workaround for a known RB bug where the baudrates 3600, 7200, 14400, and 28800 baud would not be set correctly and default to 57600 baud instead.


1.3.1: 1/11/2011
----------------
Improvements:
- Added a preferences option to automatically check for updates at startup.

Fixes:
- Fixed a bug that caused a StackOverFlowException when serial port devices were unexpectedly removed from the system, e.g. when a USB serial adapter was unplugged while the terminal was connected to that device. The error handling for this situation has been improved.
- Fixed a bug that caused an OutOfBoundsException when a serial port device failure occurred during enumeration.
- Fixed a bug that resulted in incorrect formatting of long crash reports.


1.3.0: 10/28/2010
-----------------
New features:
- Added a transmit line delay option which adds a specified delay after certain characters such as new line characters (configurable).
- Added a transmit character delay option (configurable).
- Added a "Connection/Send Break" menu item for sending serial breaks.
- Added the option to play a notification sound after a text file has been sent.
- Added auto-connect feature.
- Added the .hex file extension to the "Text Files" file type set (for the "Send Text File" dialog).
- It is now possible to have default settings loaded at startup and when a new terminal window is opened. If a default.stc settings file exists in the application folder of CoolTerm, it will be applied to new terminal windows.
- Added a menu item to save current settings as default settings.

Improvements:
- Pressing ENTER or RETURN in the connection settings dialog now envokes the "Ok" button, even if a textfield is currently selected.
- Pressing ESC in the connection settings dialog now invokes the "Cancel" button, even if a textfield is currently selected.
- Pressing Shift+ENTER or Shift+RETURN now invokes the "Send" button in "Send String" windows.
- Improved handling of command line arguments.
- The values for "Receive Buffer Size" and the character and line delays are now limited to a range from 0 to a maximum value (2,147,483,647 and 10,000, respectively).
- When a "Send String" window is opened, the text field now receives focus automatically.
- Improved exception handling and error reporting.
- Improved behavior of the command history buffer and menu.
- GUI improvements.

Fixes:
- Fixed a bug that allowed opening multiple "Save As..." for the same Terminal window dialogs on Windows.
- Fixed a bug that could cause a StackOverflow on serial port errors due to calling port.flush
- Fixed bug that could cause a crash when sending empty strings via a "Send  String" window.
- (Win) Fixed issue that would allow the terminal window to be activated via the taskbar when the connection options window is open.
- Several minor bug fixes.


1.2.0: 2/19/2010
----------------
- Added "Line Mode" to the communication settings. In "Line Mode" a line of typed text will not be sent to the serial port until the Enter key is pressed.
- Added "History" which is available in "Line Mode" the up and down arrow keys can be used to select previously typed lines.
- Added a receive buffer size limit option.
- Added handling of the bell character (ASCII code 7), which can be enabled through the communication settings.
- It is now possible to open the communication settings and edit certain options while the serial port is open.
- The viewer mode (plain or hex) is now saved as parameter in connection settings files.
- The size and position of terminal windows is now saved with connection settings.
- Fixed bug that converted occurrences CR+CR+LF strings to single spaces on Windows.


1.1.2: 7/17/2009
----------------
- Separated option to handle backspace characters in ASCII view from option to convert non-printable characters.
- Changed character used to display non-printable characters to a period (ASCII code 46) for better compatibility and consistency across platforms.
- Changed short cuts for "View/Autoscroll" and View Mode menu items to avoid conflict with other menu items such as "Edit/Select All".
- Windows build now associates .stc files with CoolTerm.
- Minor bug fixes.


1.1.1: 6/29/2009
----------------
- Added option to handle backspace characters in ASCII view to Connection Settings.
- Fixed bug in SendString that prevented typing 8 in hex mode.
- Fixed bug that printed the wrong character for cursor down key when ConvertNonPrint was enabled.
- Added a "Check for Updates" Menu item.


1.1.0: 6/18/2009
----------------
- Added an option to the connection settings to automatically terminate string sent from "Send String" windows with a configurable "Termination String", such as e.g. a linefeed etc.
- In ASCII view mode, all incoming "New Line" such as CR, LF, CR+LF, are now properly displayed as line breaks.
- Added an option to the connection settings to convert non-printable characters to generic dot characters in ASCII view.
- Added 'View' menu with menu item to switch between Hex and ASCII viewing.
- Moved 'Clear Data' menu item to 'View' menu.
- Added an 'Autoscroll' feature, accessible via the 'View' menu to enable/disable automatic scrolling of received data.
- Changed menu shortcut key for "Cycle through windows" from "0" to "`".
- Added code to produce an audible alert (system beep) when characters are typed while the serial port is not connected.
- Added a 'Help' button to the toolbar


1.0.0: 5/19/2009
----------------
- Initial Public Release




LICENSE
=======

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR HOLDERS INCLUDED IN THIS NOTICE BE LIABLE FOR ANY CLAIM, OR ANY SPECIAL INDIRECT OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
