import { mount } from "@vue/test-utils";
import Logs from "../Logs.vue";
import { describe, it, expect, beforeEach, vi } from "vitest";
import { socketProvider } from "@/plugins/socket";
import { SocketTestHelper } from "@/helper/socket-test-helper";
import { SERVER_URL } from "@/common/constants";

describe("Logs", () => {
  let wrapper: any;
  let socketTestHelper: any;

  beforeEach(() => {
    socketTestHelper = new SocketTestHelper();
    wrapper = mount(Logs, {
      global: { provide: { [socketProvider as symbol]: socketTestHelper } },
    });
  });

  it("loads logs and missions on mounted", async () => {
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.missions).toBeTruthy();
    expect(wrapper.vm.logs).toBeTruthy();
  });

  it("adds log when onLogger is called with a new log", async () => {
    const newLog = {
      id: 1,
      mission_id: 1,
      time: "2022-03-24T16:41:16.000Z",
      robot: "rover",
      category: "data",
      data: "Test log",
    };
    wrapper.vm.onLogger(JSON.stringify(newLog));
    expect(wrapper.vm.logs.length).toEqual(1);
    expect(wrapper.vm.logs[0]).toEqual(newLog);
  });

  it("loads next logs when onNext is called", async () => {
    wrapper.vm.logs = [
      {
        id: 2,
        mission_id: 1,
        time: "2022-03-24T16:41:16.000Z",
        robot: "rover",
        category: "data",
        data: "Test log",
      },
    ];
    wrapper.vm.start_id = 2;
    await wrapper.vm.onNext();
    expect(wrapper.vm.logs.length).toBeGreaterThan(0);
  });
/*
  it("loads previous logs when onPrev is called", async () => {
    wrapper.vm.logs = [
      {
        id: 10,
        mission_id: 1,
        time: "2022-03-24T16:41:16.000Z",
        robot: "rover",
        category: "data",
        data: "Test log",
      },
    ];
    wrapper.vm.start_id = 0;
    await wrapper.vm.onPrev();
    expect(wrapper.vm.logs.length).toBeGreaterThan(0);
    expect(fetch).toHaveBeenCalled();
  });*/
});
