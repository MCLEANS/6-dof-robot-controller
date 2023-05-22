pipeline{
    agent any
    stages{
        stage('Configure'){
            steps{
                sh 'echo "Configuring Build Environment"'
                sh 'sudo apt install gcc-arm-none-eabi make st-tools'
            }
        }

        stage('Build'){
            steps{
                sh 'make'
                archiveArtifacts: artifacts 'build/*', fingerprint: true
            }
        }

        stage("Upload"){
            steps{
                sh 'make flash'
            }
        }

        stage('Clean'){
            steps{
                sh 'make clean'
            }
        }

    }
}