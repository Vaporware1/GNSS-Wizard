@echo off
cd /d "C:\Users\aneil\Documents\GNSS-Wizard"
echo Current branch:
git branch --show-current
echo.
echo Adding file...
git add docs/HUD_preview_blurred.png
echo.
echo Committing...
git commit -m "Add blurred HUD preview image"
echo.
echo Pushing...
git push
echo.
echo Done! Press any key to close.
pause
