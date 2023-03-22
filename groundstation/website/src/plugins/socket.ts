import type { App, InjectionKey } from "vue";
import { io, Socket } from "socket.io-client";

const socketProvider = Symbol() as InjectionKey<Socket>;

export default {
  install: (app: App) => {
    const socket = io("http://localhost:8000", { path: "/sockets" });
    app.provide<Socket>(socketProvider, socket);
  },
};

export { socketProvider };
