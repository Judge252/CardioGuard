# This Lambda function is triggered by AWS IoT Core when a new message is published to the topic.
import json
import boto3
import logging
import time

# Setup logging
logging.getLogger().setLevel(logging.INFO)

# SageMaker and S3 clients
sagemaker = boto3.client('sagemaker-runtime')
s3 = boto3.client('s3')

# SageMaker endpoint name
ENDPOINT_NAME = 'heart-disease-endpoint'
# S3 bucket for storing results
S3_BUCKET = 'your-bucket'  # Replace with your S3 bucket name
S3_PREFIX = 'cardioGuard/results/'

def lambda_handler(event, context):
    try:
        # Log incoming IoT payload
        logging.info(f"Received IoT payload: {event}")
        
        # Convert payload to JSON string
        payload = json.dumps(event)
        
        # Invoke SageMaker endpoint
        response = sagemaker.invoke_endpoint(
            EndpointName=ENDPOINT_NAME,
            ContentType='application/json',
            Body=payload
        )
        
        # Parse response
        result = json.loads(response['Body'].read().decode())
        logging.info(f"SageMaker prediction: {result}")
        
        # Store result in S3 with patient ID
        patient_id = event.get('patient_id', 'unknown')
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        s3_key = f"{S3_PREFIX}prediction-{patient_id}-{timestamp}.json"
        s3.put_object(
            Bucket=S3_BUCKET,
            Key=s3_key,
            Body=json.dumps(result)
        )
        logging.info(f"Stored prediction in s3://{S3_BUCKET}/{s3_key}")
        
        return {
            'statusCode': 200,
            'body': json.dumps({
                'percentage_chance_heart_disease': result['percentage_chance_heart_disease'],
                'status': result['status']
            })
        }
        
    except Exception as e:
        logging.error(f"Error: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps({'error': str(e)})
        }