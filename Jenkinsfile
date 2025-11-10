pipeline {
    agent any

    environment {
        IMAGE_NAME = "ros1_ci:latest"
        CONTAINER_NAME = "ros1_ci_test"
        DISPLAY = ":0"
        QT_X11_NO_MITSHM = "1"
    }

    stages {

        stage('Checkout Code') {
            steps {
                echo "Checking out source code..."
                checkout scm
            }
        }

        stage('Build Docker Image') {
            steps {
                echo "Building Docker image ${IMAGE_NAME}..."
                sh 'docker build -t ${IMAGE_NAME} .'
            }
        }

        stage('Run Gazebo Simulation and Tests') {
            steps {
                echo "Running Docker container..."
                sh '''
                docker run --name ${CONTAINER_NAME} --rm \
                    -e DISPLAY=${DISPLAY} \
                    -e QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM} \
                    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                    ${IMAGE_NAME} /bin/bash -c "
                        set -e
                        source /opt/ros/noetic/setup.bash &&
                        source /catkin_ws/devel/setup.bash &&
                        echo 'Starting Gazebo simulation...' &&
                        roslaunch tortoisebot_gazebo tortoisebot_playground.launch &
                        GAZEBO_PID=$!

                        echo 'Waiting for /odom topic (Gazebo startup)...'
                        timeout 90 bash -c 'until rostopic list | grep -q /odom; do sleep 2; echo Waiting...; done'

                        echo 'Gazebo is ready. Running rostest...'
                        rostest tortoisebot_waypoints waypoints_test.test -r

                        echo 'Cleaning up Gazebo process...'
                        kill $GAZEBO_PID || true
                        wait $GAZEBO_PID || true
                    "
                '''
            }
        }
    }

    post {
        success {
            echo "Build and test completed successfully! Gazebo shut down cleanly."
        }
        failure {
            echo "Build or test failed. Check the Console Output above for details."
        }
        always {
            sh '''
            docker stop ${CONTAINER_NAME} || true
            docker rm ${CONTAINER_NAME} || true
            '''
        }
    }
}
