pipeline {
    agent any
    stages {
        stage('SCM') {
            steps {
                script {
                    properties([pipelineTriggers([pollSCM('* * * * *')])])
                }
                git branch: 'main', url: 'https://github.com/rg-masterclass/ros1_ci.git'
            }
        }
        stage('BUILD') {
            steps {
                sh 'cd ~/catkin_ws/src'
                sh '''
                    #!/bin/bash
                    if [ ! -d "ros1_ci" ]; then
                        git clone https://github.com/rg-masterclass/ros1_ci.git
                    else
                        cd ros1_ci
                        git pull origin main
                    fi
                    '''
                sh 'cd ~/catkin_ws/src/ros1_ci && sudo docker build . -t ros1_ci'
            }
        }
        stage('TEST') {
            steps {
                sh 'docker run --rm ros1_ci:latest "rostest tortoisebot_waypoints waypoints_test.test"'
            }
        }
    }
}