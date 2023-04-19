# Équipe 101

## Vidéos des requis
Les vidéos des requis se trouvent dans un dossier Drive accessible via le lien suivant:
https://drive.google.com/drive/folders/1XdpA0hY2CjlrFo-zKvEWFGm8XBktVE11?usp=sharing

## Groundstation

Construire le container

```
docker compose build groundstation
```

Démarrer le container

```
docker compose up --build groundstation
```

## Simulation
Construire le container
```
docker compose run --build -p 5901:5901 sim
```

Puis allez voir dans le README dans le dossier sim pour plus d'information pour démarrer la simulation 

## Convention de codage
En Python, nous utilisons PEP 8 pour standardiser le format de code à travers le projet. 

Pour TypeScript et Vue.js, nous utilisons Prettier ainsi que ESLint pour standardiser notre code.