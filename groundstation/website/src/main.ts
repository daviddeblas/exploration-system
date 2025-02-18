import { createApp } from "vue";
import { createPinia } from "pinia";

import App from "./App.vue";
import router from "./router";
import socket from "./plugins/socket";

import "./assets/css/main.css";
import "./assets/css/index.css";
import "@fortawesome/fontawesome-free/css/all.css";

const app = createApp(App);

app.use(createPinia());
app.use(router);
app.use(socket);

app.mount("#app");
