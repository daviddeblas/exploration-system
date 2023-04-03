export enum ROBOT_STATUS {
  offline = "hors ligne",
  pending = "en attente",
  in_mission = "en mission",
  crashed = "écrasé"
}

export const SERVER_URL = import.meta.env.DEV ? "http://localhost:8000" : "";
