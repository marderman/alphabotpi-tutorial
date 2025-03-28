import pickle
from sklearn import datasets
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import classification_report

# Load the digits dataset
digits = datasets.load_digits()
X = digits.data
y = digits.target

# Split dataset into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42)

# Create and train an MLPClassifier
mlp = MLPClassifier(hidden_layer_sizes=(100,), max_iter=500, random_state=42)
mlp.fit(X_train, y_train)

# Evaluate the classifier
y_pred = mlp.predict(X_test)
print(classification_report(y_test, y_pred))

# Save the trained model
with open("digit_classifier.pkl", "wb") as f:
    pickle.dump(mlp, f)

print("MLP model trained and saved as digit_classifier.pkl")
