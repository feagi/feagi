@ECHO OFF

SET ARG_COUNT=0
FOR %%x IN (%*) DO SET /A ARG_COUNT+=1
IF %ARG_COUNT% LSS 3 (
    ECHO:
    ECHO ">>> USAGE: .\run_private.bat <repo_ssh_url> <git_ssh_key_path> <branch>"
    ECHO:
    ECHO "    repo_ssh_url: SSH URL of private repository"
    ECHO "    git_ssh_key_path: Absolute file path of git SSH key"
    ECHO "    branch: Branch to checkout"
    ECHO:
    EXIT /b
)

SET PRIVATE_FEAGI_REPO=%1
SET GIT_PRIVATE_KEY_PATH=%2
SET PRIVATE_REPO_BRANCH=%3
SET DOCKER_BUILDKIT=1

SETX PRIVATE_FEAGI_REPO %1 >NUL
SETX PRIVATE_REPO_BRANCH %3 >NUL

docker image rm -f docker_feagi
docker build --no-cache --ssh default=%GIT_PRIVATE_KEY_PATH% ^
                        --build-arg REPO=%PRIVATE_FEAGI_REPO% ^
                        --build-arg BRANCH=%PRIVATE_REPO_BRANCH% ^
                        -f Dockerfile.private .
docker-compose -f feagi.yml build godot ros-gazebo
docker-compose -f feagi.yml -f feagi.private.yml up
