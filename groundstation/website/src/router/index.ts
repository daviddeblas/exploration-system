import { createRouter, createWebHistory } from "vue-router";
import Home from "../views/Home.vue";

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: "/",
      name: "home",
      component: Home,
    },
    {
      path: "/missions",
      name: "missions",
      component: () => import("../views/Missions.vue"),
    },
    {
      path: "/logs",
      name: "logs",
      component: () => import("../views/Logs.vue"),
    },
  ],
});

export default router;
