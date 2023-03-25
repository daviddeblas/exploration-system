# Installation


```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

# Lancer le serveur

```
uvicorn main:app --reload
```

# Lancer les tests

```
python3 -m unittest -v test_socket.py
```
