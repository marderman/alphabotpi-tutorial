import numpy as np
import pickle
from sklearn.datasets import fetch_openml
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score

# Step 1: Load MNIST dataset
print("Loading MNIST dataset...")
mnist = fetch_openml('mnist_784', version=1, parser="auto")
X, y = mnist.data, mnist.target.astype(int)  # Convert labels to integer

# Step 2: Normalize pixel values to range [0,1]
X = X / 255.0  # Each pixel was originally in range [0,255]

# Step 3: Split into training and test sets (80% train, 20% test)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Step 4: Train k-NN model (choosing k=3 as an example)
print("Training k-NN model...")
knn = KNeighborsClassifier(n_neighbors=3, n_jobs=-1)  # Use all CPU cores
knn.fit(X_train, y_train)

# Step 5: Evaluate accuracy on test set
y_pred = knn.predict(X_test)
accuracy = accuracy_score(y_test, y_pred)
print(f"Test Accuracy: {accuracy:.4f}")

# Step 6: Save trained model to file
with open("knn_model.pkl", "wb") as f:
    pickle.dump(knn, f)

print("Model training complete. Saved as knn_model.pkl")
