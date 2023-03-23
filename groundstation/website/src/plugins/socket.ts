import type { App, InjectionKey } from "vue";
import { io, Socket } from "socket.io-client";
import { SERVER_URL } from "@/common/constants";

const socketProvider = Symbol() as InjectionKey<Socket>;

export default {
  install: (app: App) => {
    const socket = io(SERVER_URL, { path: "/sockets" });
    app.provide<Socket>(socketProvider, socket);
  },
};

export { socketProvider };
