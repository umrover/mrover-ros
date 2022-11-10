<template>
<div>
    <button v-on:click="download_csv_file(spectral_data)">Generate Report</button>
</div>
</template>

<script>
export default {
    data() {
        return {
            csvFileData: []
        }
    },
    props: {
        spectral_data: {
            type: Array,
            required: true
        },
    },
    methods: {
        download_csv_file: function (spectral_data) {
            const time = new Date(Date.now())
            const timeString = time.toTimeString().substring(0, 17) + " " + time.toDateString()
            this.csvFileData = [
                [timeString, (spectral_data[0]).toFixed(0)],
                [timeString, (spectral_data[1]).toFixed(0)],
                [timeString, (spectral_data[2]).toFixed(0)],
                [timeString, (spectral_data[3]).toFixed(0)],
                [timeString, (spectral_data[4]).toFixed(0)],
                [timeString, (spectral_data[5]).toFixed(0)]
            ];
            //define the heading for each row of the data
            var csv = 'Timestamp,Spectral 0,Spectral 1,Spectral 2\n';
            //merge the data with CSV
            this.csvFileData.forEach(function (row) {
                csv += row.join(',');
                csv += "\n";
            });
            var hiddenElement = document.createElement('a');
            hiddenElement.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv);
            hiddenElement.target = '_blank';
            //provide the name for the CSV file to be downloaded
            let report_name = prompt("Enter filename:", timeString);
            hiddenElement.download = report_name + '.csv';
            hiddenElement.click();
        }
    }
}
</script>

<style scoped>
button {
    width: 17vh;
    height: 7vh;
}
</style>
