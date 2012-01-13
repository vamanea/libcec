@echo off

IF EXIST "%ProgramFiles%\NSIS\makensis.exe" (
  set NSIS="%ProgramFiles%\NSIS\makensis.exe"
) ELSE IF EXIST "%ProgramFiles(x86)%\NSIS\makensis.exe" (
  set NSIS="%ProgramFiles(x86)%\NSIS\makensis.exe"
) ELSE GOTO NONSIS

IF NOT EXIST "C:\WinDDK\7600.16385.1" GOTO NODDK
set DDK="C:\WinDDK\7600.16385.1"

IF "%VS100COMNTOOLS%"=="" (
  set COMPILER10="%ProgramFiles%\Microsoft Visual Studio 10.0\Common7\IDE\VCExpress.exe"
) ELSE IF EXIST "%VS100COMNTOOLS%\..\IDE\VCExpress.exe" (
  set COMPILER10="%VS100COMNTOOLS%\..\IDE\VCExpress.exe"
) ELSE IF EXIST "%VS100COMNTOOLS%\..\IDE\devenv.exe" (
  set COMPILER10="%VS100COMNTOOLS%\..\IDE\devenv.exe"
)

echo Cleaning libCEC
%COMPILER10% libcec.sln /clean Release
echo Compiling libCEC
%COMPILER10% libcec.sln /build Release /project libcec
echo Compiling cec-client
%COMPILER10% libcec.sln /build Release /project testclient
echo Compiling LibCecSharp
%COMPILER10% libcec.sln /build Release /project LibCecSharp

IF "%VS90COMNTOOLS%"=="" (
  set COMPILER9="%ProgramFiles%\Microsoft Visual Studio 9.0\Common7\IDE\VCExpress.exe"
) ELSE IF EXIST "%VS90COMNTOOLS%\..\IDE\VCExpress.exe" (
  set COMPILER9="%VS90COMNTOOLS%\..\IDE\VCExpress.exe"
) ELSE IF EXIST "%VS90COMNTOOLS%\..\IDE\devenv.exe" (
  set COMPILER9="%VS90COMNTOOLS%\..\IDE\devenv.exe"
) ELSE GOTO NOSDK9

echo Compiling LibCecSharp.Net2
%COMPILER9% LibCecSharp.Net2.sln /build Release

:NOSDK9
echo Copying driver installer
copy "%DDK%\redist\DIFx\dpinst\MultiLin\amd64\dpinst.exe" ..\dpinst-amd64.exe
copy "%DDK%\redist\DIFx\dpinst\MultiLin\x86\dpinst.exe" ..\dpinst-x86.exe

echo Creating the installer
%NSIS% /V1 /X"SetCompressor /FINAL lzma" "libCEC.nsi"

echo The installer can be found here: libCEC-installer.exe

GOTO EXIT

:NOSIS
echo NSIS could not be found on your system.
GOTO EXIT

:NODDK
echo Windows DDK could not be found on your system

:EXIT