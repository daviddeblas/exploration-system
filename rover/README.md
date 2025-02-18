# Installation

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

# Utilisation sur le limo

## Créer l'image pour arm64

```
docker buildx build --platform linux/arm64 -t inf3995-101-rover .
```

## Téléverser l'image

```
docker save inf3995-101-rover | ssh -C agilex@<ip> docker load
```

## Exécuter le container

```
docker run --rm -it --network host --privileged -v /dev:/dev -e "COGNIFLY_HOSTNAME=CogniflyX.lan" --name inf3995-101-rover reg.zdev.ca/inf3995-101-rover
```

où CogniflyX correspond au nom du Cognifly utilisé

## Arrêter le container

```
docker stop inf3995-101-rover
```

# Installation des dépendances pour les tests

```
pip install -r requirements.txt
```

# Lancer les tests

```
python3 -m unittest -v test_cognifly_movement.py
```
