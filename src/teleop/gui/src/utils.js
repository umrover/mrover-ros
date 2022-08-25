class Toggle {
    constructor(init_state) {
        this.toggle = init_state
        this.previous = false
        this.input = false
        this.last_input = false
    }

    new_reading(reading) {
        this.input = reading
        if(this.input && !this.last_input) {
            //Just pushed
            this.last_input = true
            this.toggle = !this.toggle
        } else if(!this.input && this.last_input) {
            //Just released
            this.last_input = false;
        }

        this.previous = reading;
        return this.toggle;
    }
}

const convertDMS = function(coord_in, odom_format) {
    const DEG_DECIMALS = 8;
    const MIN_DECIMALS = 6;
    const SEC_DECIMALS = 4;
    const coord_total = coord_in.d + coord_in.m/60 + coord_in.s/3600;
    let coord_out = {};
    if (odom_format === 'DMS') {
        coord_out.d = Math.trunc(coord_total.toFixed(DEG_DECIMALS));
        coord_out.m = Math.trunc(((coord_total - coord_out.d ) * 60).toFixed(MIN_DECIMALS));
        coord_out.s = +(((coord_total - coord_out.d ) * 60 - coord_out.m) * 60).toFixed(SEC_DECIMALS);
    } else if (odom_format === 'DM') {
        coord_out.d = Math.trunc(coord_total.toFixed(DEG_DECIMALS));
        coord_out.m = +((coord_total - coord_out.d ) * 60).toFixed(MIN_DECIMALS);
        coord_out.s = 0;
    } else {
        coord_out.d = +(coord_total).toFixed(DEG_DECIMALS);
        coord_out.m = 0;
        coord_out.s = 0;
    }
    return coord_out;
}

export {Toggle, convertDMS}
