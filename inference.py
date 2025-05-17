# This script is used for inference with a trained XGBoost model for heart disease prediction.
import json
import pandas as pd
import numpy as np
import joblib
import xgboost as xgb
import os

def model_fn(model_dir):
    # Load the trained XGBoost model
    model = xgb.Booster()
    model.load_model(os.path.join(model_dir, 'xgboost_heart_disease_model.json'))
    
    # Load the scaler
    scaler = joblib.load(os.path.join(model_dir, 'scaler.joblib'))
    
    return {'model': model, 'scaler': scaler}

def input_fn(request_body, request_content_type):
    if request_content_type == 'application/json':
        data = json.loads(request_body)
        return data
    else:
        raise ValueError(f"Unsupported content type: {request_content_type}")

def predict_fn(input_data, model):
    # Extract features from IoT payload
    feature_map = {
        'heartRateBPM': 'thalach',
        'age': 'age',
        'sex': 'sex',
        'cp': 'cp',
        'restecg': 'restecg',
        'oldpeak': 'oldpeak',
        'slope': 'slope'
    }
    
    # Create DataFrame with required features
    features = ['age', 'sex', 'cp', 'restecg', 'thalach', 'oldpeak', 'slope']
    data = {}
    for payload_key, feature in feature_map.items():
        if payload_key not in input_data:
            raise ValueError(f"Missing feature: {payload_key}")
        data[feature] = [input_data[payload_key]]
    
    df = pd.DataFrame(data, columns=features)
    
    # Normalize numeric features
    numeric_features = ['age', 'thalach', 'oldpeak']
    df[numeric_features] = model['scaler'].transform(df[numeric_features])
    
    # Convert to DMatrix for XGBoost
    dmatrix = xgb.DMatrix(df)
    
    # Predict probability of heart disease (target = 0)
    prob_heart_disease = model['model'].predict(dmatrix)[0]
    percentage_heart_disease = (1 - prob_heart_disease) * 100  # Convert to percentage of target = 0
    
    # Determine status
    status = 'Unhealthy' if percentage_heart_disease >= 50 else 'Healthy'
    
    return {
        'percentage_chance_heart_disease': round(percentage_heart_disease, 2),
        'status': status
    }

def output_fn(prediction, content_type):
    if content_type == 'application/json':
        return json.dumps(prediction)
    else:
        raise ValueError(f"Unsupported content type: {content_type}")