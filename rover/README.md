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
docker run -it --rm --network host --device /dev/snd inf3995-101-rover
```
