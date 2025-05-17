import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, roc_auc_score
import xgboost as xgb
import joblib
import os
import argparse

def train_model(data_path, output_path):
    # Load dataset
    df = pd.read_csv(data_path)
    print("Dataset Info:")
    print(df.info())

    # Define features and target
    features = ['age', 'sex', 'cp', 'restecg', 'thalach', 'oldpeak', 'slope']
    X = df[features]
    y = df['target']

    # Handle missing values
    X = X.fillna(X.median())

    # Normalize numeric features
    scaler = StandardScaler()
    numeric_features = ['age', 'thalach', 'oldpeak']
    X[numeric_features] = scaler.fit_transform(X[numeric_features])

    # Save scaler for inference
    joblib.dump(scaler, os.path.join(output_path, 'scaler.joblib'))

    # Split data
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)

    # Initialize and train XGBoost model
    xgb_model = xgb.XGBClassifier(
        objective='binary:logistic',
        eval_metric='logloss',
        random_state=42,
        max_depth=5,
        learning_rate=0.1,
        n_estimators=200,
        subsample=0.8,
        colsample_bytree=0.8
    )
    xgb_model.fit(X_train, y_train)

    # Evaluate model
    y_pred = xgb_model.predict(X_test)
    y_pred_proba = xgb_model.predict_proba(X_test)[:, 1]
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred))
    print("\nROC-AUC Score:", roc_auc_score(y_test, y_pred_proba))

    # Save model
    xgb_model.save_model(os.path.join(output_path, 'xgboost_heart_disease_model.json'))

if _name_ == '_main_':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-path', type=str, default='/opt/ml/input/data/training/heart.csv')
    parser.add_argument('--output-path', type=str, default='/opt/ml/model')
    args = parser.parse_args()

    train_model(args.data_path, args.output_path)