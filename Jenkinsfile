pipeline{
    agent any
    environment {
        PATH = "$PATH:/opt/xpack-arm-none-eabi-gcc-10.3.1-2.2/bin/"
    }
    
    stages{
        stage('Config'){
            steps{
                sh 'echo "Configuring Build Environment"'
                sh 'make --version'
            }
        }

        stage('Build'){
            steps{
                sh 'make'
            }
        }
    }
}
