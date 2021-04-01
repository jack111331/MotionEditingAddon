del /f MotionEditingAddon.zip
cd ..
tar.exe  --exclude=*.zip  --exclude=MotionEditingAddon/.git/* --exclude=MotionEditingAddon/__pycache__/* -a -c -f MotionEditingAddon.zip MotionEditingAddon/*
move MotionEditingAddon.zip MotionEditingAddon/