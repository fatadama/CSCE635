
## Note on installing nanomsg-python on Windows
(Windows 7, 64-bit, with 32-bit Python 2.7 used. Visual Studio Community 2015 used)
Download nanomsg from its github (https://github.com/nanomsg/nanomsg) and install following windows instructions (http://nanomsg.org/download.html)
Copy build file nanomsg.lib from build directory (probably /Debug or /Release) to PYTHON_DIRECTORY\libs, where PYTHON_DIRECTORY is where your python.exe is located (eg. C:\Python27)
Copy source files *.h from /src to PYTHON_DIRECTORY\include\nanomsg
Download nanomsg-python from its github (https://github.com/tonysimpson/nanomsg-python).
From command line run "python setup.py install" in nanomsg-python folder.
From python, you should be able to import nanomsg IF nanomsg.dll from the build folder of nanomsg is on the path.
Directions followed from this thread: (https://github.com/tonysimpson/nanomsg-python/issues/5#issuecomment-112679235)
