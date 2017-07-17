sudo kextunload -b io.sinetek.macRTSX

xcodebuild -configuration Debug &&
sudo cp -R build/Debug/macRTSX.kext /tmp &&
sudo chown -R root:wheel /tmp/macRTSX.kext &&
sudo kextutil /tmp/macRTSX.kext;
