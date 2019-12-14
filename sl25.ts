//% color=#444444 icon="\uf0eb"
namespace SL26 {

    export enum DistanceMode {
        //% block="short"
        Short = 0,
        //% block="medium"
        Medium = 1,
        //% block="long"
        Long = 2,
        //% block="unknown"
        Unknown = 3
    }

    //%block="SL25 initialize"
    //%shim=SL25::init
    export function init(): void {
        return;
    }

    //%block="SL25 setDistanceMode %u"
    //%shim=SL25::setDistanceMode
    export function setDistanceMode(u: DistanceMode): number {
        return 1;
    }

    //%block="SL25 setMeasurementTimingBudget"
    //%shim=SL25::setMeasurementTimingBudget
    export function setMeasurementTimingBudget(timing: number): void {
        return;
    }

    //%block="SL25 startContinuous"
    //%shim=SL25::startContinuous
    export function startContinuous(x: number): void {
        return;
    }

    //%block="SL25 disable power"
    //%shim=SL25::disablePower
    export function disbalePower(): void {
        return;
    }

    //%block="SL25 read"
    //%shim=SL25::read
    export function read(): void {
        return;
    }
}
